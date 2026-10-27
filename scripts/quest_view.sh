#!/usr/bin/env bash
# Derived from KUKU-Robot-Lab/motion_acq scripts/quest_usb.sh at 01dc218 (Apache-2.0).
# Meta Quest link for quest_view: the headset opens the quest_view page (WebXR) over adb.
#
#   scripts/quest_view.sh status    # adb device and the page link
#   scripts/quest_view.sh view      # open the page (quest_view server must run) and enter VR
#   scripts/quest_view.sh page      # the same page, without entering VR
#   scripts/quest_view.sh vr        # (re)enter VR on the open page from the PC (an asleep headset is woken
#                                   # with the proximity sensor off first); vr --status = page state
#   scripts/quest_view.sh unworn    # keep the headset awake off the head (proximity sensor off), wake it
#   scripts/quest_view.sh worn      # proximity sensor back to normal (also after a Quest reboot)
#   scripts/quest_view.sh wifi      # (cable in, once per Quest boot) the same link over Wi-Fi; then the
#                                   # cable may come out. wifi-off: back to the cable.
#
# The page reaches quest_view as http://localhost:8787 through adb reverse (a secure context for
# WebXR, no certificate). quest_view re-publishes the headset poses on TCP 127.0.0.1:65432, where
# ros_bridge (quest.source: quest_view) reads them.
#
# Wi-Fi: adb over TCP 5555 to the headset's Wi-Fi address, remembered in ~/.config/macq/quest_wifi;
# every command then talks to that device. More latency / jitter than USB, a dropped link stops
# the arms (tracking lost), and the headset runs on its battery.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ADB="${ADB:-$(command -v adb || echo "$HOME/opt/platform-tools/adb")}"

die() { echo "error: $*" >&2; exit 1; }

[[ -x "$ADB" ]] || die "adb not found (expected $HOME/opt/platform-tools/adb)."

WIFI_FILE="${MACQ_QUEST_WIFI_FILE:-$HOME/.config/macq/quest_wifi}"
WIFI_PORT=5555

devices() { "$ADB" devices | awk 'NR > 1 && NF >= 2 {print $1, $2}'; }

pick_serial() {
  # the remembered Wi-Fi device (dialled again when adb lost it), else USB (as motion_acq.quest_view.link)
  local list saved
  list="$(devices)"
  saved="$(cat "$WIFI_FILE" 2>/dev/null || true)"
  if [[ -n "$saved" ]] && ! grep -qx "$saved device" <<< "$list"; then
    "$ADB" connect "$saved" >/dev/null 2>&1 || true
    list="$(devices)"
  fi
  if [[ -n "$saved" ]] && grep -qx "$saved device" <<< "$list"; then echo "$saved"; return; fi
  awk '$2 == "device" && $1 !~ /:/ {print $1; exit}' <<< "$list" | grep . && return
  awk '$2 == "device" {print $1; exit}' <<< "$list" | grep . && return
  awk 'NR == 1 {print $1}' <<< "$list"
}

one_device() {
  local serial state saved
  serial="$(pick_serial)"
  saved="$(cat "$WIFI_FILE" 2>/dev/null || true)"
  [[ -n "$serial" ]] || die "no Quest on USB${saved:+ or Wi-Fi ($saved)}. Plug it in and accept 'Allow USB debugging' in the headset${saved:+ (after a Quest reboot: cable in, then $0 wifi again)}."
  state="$(devices | awk -v s="$serial" '$1 == s {print $2}')"
  case "$state" in
    device) ;;
    unauthorized) die "Quest is unauthorized: accept 'Allow USB debugging' inside the headset." ;;
    no*) die "no USB permission for the Quest: a udev rule for vendor 2833 (plugdev) is needed." ;;
    *) die "unexpected adb state: $state ($serial)" ;;
  esac
  export ANDROID_SERIAL="$serial"
}

cmd_wifi() {
  local usb ip target had_reverse=0 i
  usb="$(devices | awk '$2 == "device" && $1 !~ /:/ {print $1; exit}')"
  [[ -n "$usb" ]] || die "the switch to Wi-Fi needs the cable once: plug the Quest in (USB debugging allowed), then $0 wifi"
  ip="$("$ADB" -s "$usb" shell ip -f inet addr show wlan0 | awk '/inet /{sub(/\/.*/, "", $2); print $2; exit}')"
  [[ -n "$ip" ]] || die "the Quest has no Wi-Fi address: join the same Wi-Fi as this PC in the headset"
  target="$ip:$WIFI_PORT"
  # 10.12: the Quest on another router with the same 192.168.1.x range looks reachable but is not.
  # Stop before 'tcpip', which restarts adbd and drops the cable's page link.
  local quest_ssid pc_ssids
  quest_ssid="$("$ADB" -s "$usb" shell cmd wifi status 2>/dev/null | sed -n 's/^Wifi is connected to "\(.*\)"\r\{0,1\}$/\1/p' | head -1)"
  pc_ssids="$("${NMCLI:-nmcli}" -t -f ACTIVE,SSID dev wifi 2>/dev/null | awk -F: '$1 == "yes" {print $2}' | sort -u)"
  if [[ -n "$quest_ssid" && -n "$pc_ssids" ]] && ! grep -qxF "$quest_ssid" <<< "$pc_ssids" \
     && ! "${PING:-ping}" -c 1 -W 1 "$ip" >/dev/null 2>&1; then
    die "the Quest is on Wi-Fi '$quest_ssid' but this PC is on '$(tr '\n' ' ' <<< "$pc_ssids" | sed 's/ $//')' and cannot reach $ip: join the PC's Wi-Fi in the headset, then $0 wifi again (the cable link is unchanged)"
  fi
  # adbd restarts in TCP mode and drops the links of the cable: set them again on the Wi-Fi device
  "$ADB" -s "$usb" reverse --list | grep -q "tcp:$VIEW_PORT" && had_reverse=1
  "$ADB" -s "$usb" tcpip "$WIFI_PORT" >/dev/null
  for i in 1 2 3 4 5 6 7 8; do
    sleep 1
    "$ADB" connect "$target" 2>&1 | grep -q "connected to" && break
  done
  devices | grep -qx "$target device" || die "could not reach $target over Wi-Fi (same network as this PC? $(ip -4 -o addr show scope global | awk '{print $4}' | tr '\n' ' '))"
  mkdir -p "$(dirname "$WIFI_FILE")"
  echo "$target" > "$WIFI_FILE"
  export ANDROID_SERIAL="$target"
  if (( had_reverse )); then "$ADB" reverse "tcp:$VIEW_PORT" "tcp:$VIEW_PORT" >/dev/null; echo "page link (reverse $VIEW_PORT) moved to Wi-Fi"; fi
  echo "Wi-Fi link: $target. The cable may come out now; the headset page reconnects by itself."
  echo "After a Quest reboot: cable in, then $0 wifi again."
}

cmd_wifi_off() {
  local saved usb
  saved="$(cat "$WIFI_FILE" 2>/dev/null || true)"
  rm -f "$WIFI_FILE"
  [[ -n "$saved" ]] && "$ADB" disconnect "$saved" >/dev/null 2>&1 || true
  usb="$(devices | awk '$2 == "device" && $1 !~ /:/ {print $1; exit}')"
  if [[ -n "$usb" ]]; then
    "$ADB" -s "$usb" usb >/dev/null || true  # adbd back to USB only (restarts: reopen the page)
    echo "back to the cable ($usb): reopen the page ($0 page)"
  else
    echo "Wi-Fi link forgotten; plug the cable in"
  fi
}

cmd_status() {
  "$ADB" devices -l
  one_device
  echo "link: $ANDROID_SERIAL ($([[ "$ANDROID_SERIAL" == *:* ]] && echo Wi-Fi || echo USB))"
  echo "reverse (page link):"; "$ADB" reverse --list
}




VIEW_PORT="${QUEST_VIEW_PORT:-8787}"

cmd_page() {
  # The page reaches macq quest-view as http://localhost (adb reverse: a secure
  # context for WebXR, no certificate); quest-view itself serves port 65432.
  one_device
  "$ADB" reverse "tcp:$VIEW_PORT" "tcp:$VIEW_PORT"
  "$ADB" shell am start -a android.intent.action.VIEW -d "http://localhost:$VIEW_PORT" com.oculus.browser >/dev/null
  echo "Quest Browser opened http://localhost:$VIEW_PORT"
}

cmd_view() {
  cmd_page
  echo "entering VR from here ..."
  cmd_vr || echo "VR did not start from the PC; press 'VR 시작' in the headset (first time: allow VR)."
}

cmd_vr() {
  # WebXR needs a user gesture in the page; the browser devtools give it to the PC.
  one_device
  if [[ "${1:-}" != "--status" ]] && "$ADB" shell dumpsys power | grep -q "mWakefulness=Asleep"; then
    # 10.06 user: the headset is used off the head as well: wake it with the proximity sensor off
    echo "the headset is asleep (not worn): proximity sensor off and wake it ..."
    cmd_unworn
  fi
  "$ROOT/.venv/bin/python" -m openarm_quest_view.devtools --view-port "$VIEW_PORT" "$@"
}

cmd_unworn() {
  one_device
  "$ADB" shell am broadcast -a com.oculus.vrpowermanager.prox_close >/dev/null
  "$ADB" shell input keyevent KEYCODE_WAKEUP
  sleep 1
  if "$ADB" shell dumpsys power | grep -q "mWakefulness=Awake"; then
    echo "proximity sensor off: the headset stays awake off the head (undo: $0 worn, or a Quest reboot)"
  else
    die "the headset did not wake: press its power button once, then run $0 unworn again"
  fi
}

cmd_worn() {
  one_device
  "$ADB" shell am broadcast -a com.oculus.vrpowermanager.automation_disable >/dev/null
  echo "proximity sensor back to normal: the headset sleeps when taken off"
}


case "${1:-status}" in
  status) cmd_status ;;
  view) cmd_view ;;
  page) cmd_page ;;
  vr) shift; cmd_vr "$@" ;;
  unworn) cmd_unworn ;;
  worn) cmd_worn ;;
  wifi) cmd_wifi ;;
  wifi-off) cmd_wifi_off ;;
  *) die "usage: $0 {status|view|page|vr|unworn|worn|wifi|wifi-off}" ;;
esac

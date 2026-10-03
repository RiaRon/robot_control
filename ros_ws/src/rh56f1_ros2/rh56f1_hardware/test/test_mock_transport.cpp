// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <cmath>
#include <limits>

#include "gmock/gmock.h"
#include "rh56f1_hardware/mock_transport.hpp"

namespace rh56f1_hardware {
namespace {

using ::testing::Each;
using ::testing::Eq;

ActuatorTargets Filled(double value) {
  ActuatorTargets t;
  t.fill(value);
  return t;
}

TEST(MockTransport, StartsDisconnectedAndConnectSucceeds) {
  MockTransport transport("right");
  EXPECT_FALSE(transport.is_connected());
  EXPECT_TRUE(transport.connect());
  EXPECT_TRUE(transport.is_connected());
  EXPECT_EQ(transport.last_error(), TransportError::kNone);
}

TEST(MockTransport, WriteBeforeConnectFailsWithNotConnected) {
  MockTransport transport("right");
  EXPECT_FALSE(transport.write_positions(Filled(0.1)));
  EXPECT_EQ(transport.last_error(), TransportError::kNotConnected);
}

TEST(MockTransport, ReadBeforeConnectFailsWithNotConnected) {
  MockTransport transport("right");
  ActuatorStates out;
  EXPECT_FALSE(transport.read_state(out));
  EXPECT_EQ(transport.last_error(), TransportError::kNotConnected);
}

TEST(MockTransport, RejectsNonFiniteCommandsWithoutTouchingLastWritten) {
  MockTransport transport("right");
  ASSERT_TRUE(transport.connect());
  ASSERT_TRUE(transport.write_positions(Filled(0.2)));

  ActuatorTargets bad = Filled(0.2);
  bad[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(transport.write_positions(bad));
  EXPECT_EQ(transport.last_error(), TransportError::kInvalidCommand);

  bad = Filled(0.2);
  bad[4] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(transport.write_positions(bad));
  EXPECT_EQ(transport.last_error(), TransportError::kInvalidCommand);

  // The rejected commands must not have overwritten the last accepted one.
  EXPECT_THAT(transport.last_written(), Each(Eq(0.2)));
}

TEST(MockTransport, WriteThenReadTracksInstantlyByDefault) {
  MockTransport transport("left");
  ASSERT_TRUE(transport.connect());
  ActuatorTargets target = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  ASSERT_TRUE(transport.write_positions(target));

  ActuatorStates out;
  ASSERT_TRUE(transport.read_state(out));
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    EXPECT_DOUBLE_EQ(out[i].position_rad, target[i]);
  }
}

TEST(MockTransport, LagFractionOneNeverMoves) {
  MockTransport transport("left");
  transport.set_lag_fraction(1.0);
  ASSERT_TRUE(transport.connect());
  ASSERT_TRUE(transport.write_positions(Filled(0.5)));

  ActuatorStates out;
  ASSERT_TRUE(transport.read_state(out));
  EXPECT_THAT(out, Each(::testing::Field(&ActuatorState::position_rad, 0.0)));
}

TEST(MockTransport, InjectFailureFiresExactlyOnceThenRecovers) {
  MockTransport transport("right");
  ASSERT_TRUE(transport.connect());
  transport.inject_failure(TransportError::kCommunicationError, 1);

  EXPECT_FALSE(transport.write_positions(Filled(0.1)));
  EXPECT_EQ(transport.last_error(), TransportError::kCommunicationError);

  // The injection was consumed; the same call now succeeds.
  EXPECT_TRUE(transport.write_positions(Filled(0.1)));
}

TEST(MockTransport, InjectFailureCanCoverMultipleCalls) {
  MockTransport transport("right");
  ASSERT_TRUE(transport.connect());
  transport.inject_failure(TransportError::kTimeout, 3);

  ActuatorStates out;
  EXPECT_FALSE(transport.read_state(out));
  EXPECT_FALSE(transport.write_positions(Filled(0.0)));
  EXPECT_FALSE(transport.read_state(out));
  EXPECT_TRUE(transport.write_positions(Filled(0.0)));  // 4th call: injection exhausted
}

TEST(MockTransport, PermanentTimeoutBlocksEveryCallUntilReconnect) {
  MockTransport transport("right");
  ASSERT_TRUE(transport.connect());
  transport.inject_permanent_timeout();

  ActuatorStates out;
  EXPECT_FALSE(transport.write_positions(Filled(0.0)));
  EXPECT_EQ(transport.last_error(), TransportError::kTimeout);
  EXPECT_FALSE(transport.read_state(out));
  EXPECT_EQ(transport.last_error(), TransportError::kTimeout);
  // Still "connected" (the port is open) — this simulates a device that has
  // gone silent, not a severed link.
  EXPECT_TRUE(transport.is_connected());

  transport.disconnect();
  ASSERT_TRUE(transport.connect());
  EXPECT_TRUE(transport.write_positions(Filled(0.0)));
}

TEST(MockTransport, OptionalStatePassesThroughWhenSet) {
  MockTransport transport("right");
  ASSERT_TRUE(transport.connect());
  transport.set_reported_current(0, 1.23);
  transport.set_reported_temperature(1, 45.6);
  transport.set_device_error(2, true);

  ActuatorStates out;
  ASSERT_TRUE(transport.read_state(out));
  EXPECT_TRUE(out[0].current_available);
  EXPECT_DOUBLE_EQ(out[0].current_a, 1.23);
  EXPECT_TRUE(out[1].temperature_available);
  EXPECT_DOUBLE_EQ(out[1].temperature_c, 45.6);
  EXPECT_TRUE(out[2].device_error);
  EXPECT_FALSE(out[0].device_error);
}

TEST(MockTransport, CallCountersTrackInvocations) {
  MockTransport transport("right");
  transport.connect();
  transport.connect();
  ActuatorStates out;
  transport.read_state(out);
  transport.write_positions(Filled(0.0));
  transport.write_positions(Filled(0.0));
  transport.write_positions(Filled(0.0));

  EXPECT_EQ(transport.connect_call_count(), 2);
  EXPECT_EQ(transport.read_call_count(), 1);
  EXPECT_EQ(transport.write_call_count(), 3);
}

TEST(TransportErrorToString, CoversEveryEnumerator) {
  EXPECT_STREQ(to_string(TransportError::kNone), "none");
  EXPECT_STREQ(to_string(TransportError::kNotConnected), "not_connected");
  EXPECT_STREQ(to_string(TransportError::kTimeout), "timeout");
  EXPECT_STREQ(to_string(TransportError::kCommunicationError), "communication_error");
  EXPECT_STREQ(to_string(TransportError::kInvalidResponse), "invalid_response");
  EXPECT_STREQ(to_string(TransportError::kInvalidCommand), "invalid_command");
  EXPECT_STREQ(to_string(TransportError::kNotImplemented), "not_implemented");
}

}  // namespace
}  // namespace rh56f1_hardware

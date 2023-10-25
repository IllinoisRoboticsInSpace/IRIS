# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
from motor_driver.generated import example_pb2

def test_example():
    print("Intermediate printed messages\n")
    print("Run tests with command:\n colcon test --packages-select motor_driver --event-handlers console_cohesion+")
    message = example_pb2.Types()
    message.signed_integer_32 = 10

    assert 10 == message.signed_integer_32, f"int32 should be 10"
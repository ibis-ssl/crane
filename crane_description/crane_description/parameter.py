# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters


# https://github.com/ros2/ros2cli/blob/780923c046f8e537e884d18bef33a2338f2d409c/ros2param/ros2param/api/__init__.py#L174
def list_parameters(node, timeout_sec=10.0):
    # create client
    client = node.create_client(ListParameters, "crane_description/list_parameters")

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec)
    if not ready:
        raise RuntimeError("Wait for service timed out")

    request = ListParameters.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    if response is None:
        raise RuntimeError("Failed to get the list of parameters")

    return response.result.names


# https://github.com/ros2/ros2cli/blob/780923c046f8e537e884d18bef33a2338f2d409c/ros2param/ros2param/api/__init__.py#L122
def get_parameters(node, parameter_names, timeout_sec=10.0):
    # create client
    client = node.create_client(GetParameters, "crane_description/get_parameters")

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec)
    if not ready:
        raise RuntimeError("Wait for service timed out")

    request = GetParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError("Failed to get parameters form node crane_description")

    return_values = {}

    # cspell: ignore pvalue
    # https://github.com/ros2/ros2cli/blob/780923c046f8e537e884d18bef33a2338f2d409c/ros2param/ros2param/api/__init__.py#L54
    for i, pvalue in enumerate(response.values):
        if pvalue.type == ParameterType.PARAMETER_BOOL:
            value = pvalue.bool_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER:
            value = pvalue.integer_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
            value = pvalue.double_value
        elif pvalue.type == ParameterType.PARAMETER_STRING:
            value = pvalue.string_value
        elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
            value = pvalue.byte_array_value
        elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
            value = pvalue.bool_array_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            value = pvalue.integer_array_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            value = pvalue.double_array_value
        elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
            value = pvalue.string_array_value
        elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            raise RuntimeError("Unknown parameter type {pvalue.type}".format_map(locals()))
        return_values[parameter_names[i]] = value

    return return_values

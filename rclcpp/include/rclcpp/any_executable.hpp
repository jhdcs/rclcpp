// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__ANY_EXECUTABLE_HPP_
#define RCLCPP__ANY_EXECUTABLE_HPP_

#include <memory>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{

enum ExType : uint8_t {
  EmptyExec,
  SubscriptionExec,
  TimerExec,
  ServiceExec,
  ClientExec,
  WaitExec,
};

struct TaggedExecutable {
  union ExecutablePtr
  {
    struct empty_type{ExType type; int empty;} empty;
    struct subscription_type{ExType type; rclcpp::SubscriptionBase::SharedPtr ptr;} subscription;
    struct timer_type{ExType type; rclcpp::TimerBase::SharedPtr ptr;} timer;
    struct service_type{ExType type; rclcpp::ServiceBase::SharedPtr ptr;} service;
    struct client_type{ExType type; rclcpp::ClientBase::SharedPtr ptr;} client;
    struct waitable_type{ExType type; rclcpp::Waitable::SharedPtr ptr;} waitable;

    ExecutablePtr(): empty{ExType::EmptyExec, 0} {};
    ExecutablePtr(const rclcpp::SubscriptionBase::SharedPtr sub): subscription{ExType::SubscriptionExec, sub} {};
    ExecutablePtr(const rclcpp::TimerBase::SharedPtr t): timer{ExType::TimerExec, t} {};
    ExecutablePtr(const rclcpp::ServiceBase::SharedPtr srv): service{ExType::ServiceExec, srv} {};
    ExecutablePtr(const rclcpp::ClientBase::SharedPtr c): client{ExType::ClientExec, c} {};
    ExecutablePtr(const rclcpp::Waitable::SharedPtr w): waitable{ExType::WaitExec, w} {};
    ExecutablePtr(ExecutablePtr const& other) {
      // This should be safe
      switch (other.subscription.type)
      {
      case ExType::SubscriptionExec:
        ::new(&subscription) auto(&other.subscription);
        break;
      case ExType::TimerExec:
        ::new(&timer) auto(&other.timer);
        break;
      case ExType::ServiceExec:
        ::new(&service) auto(&other.service);
        break;
      case ExType::ClientExec:
        ::new(&client) auto(&other.client);
        break;
      case ExType::WaitExec:
        ::new(&waitable) auto(other.waitable);
        break;
      case ExType::EmptyExec:
        // Why would anyone want to do this?
        ::new(&empty) auto(other.empty);
        break;
      }
    };

    ~ExecutablePtr() {
      if (subscription.type == ExType::SubscriptionExec) {
        subscription.~subscription_type();
      } else if (timer.type == ExType::TimerExec) {
        timer.~timer_type();
      } else if (service.type == ExType::ServiceExec) {
        service.~service_type();
      } else if (client.type == ExType::ClientExec) {
        client.~client_type();
      } else if (waitable.type == ExType::WaitExec) {
        waitable.~waitable_type();
      } else {
        // it's empty
        empty.~empty_type();
      }
    };
  } ptr;

  TaggedExecutable(): ptr() {};
  TaggedExecutable(rclcpp::SubscriptionBase::SharedPtr sub): ptr(sub) {};
  TaggedExecutable(rclcpp::TimerBase::SharedPtr t): ptr(t) {};
  TaggedExecutable(rclcpp::ServiceBase::SharedPtr srv): ptr(srv) {};
  TaggedExecutable(rclcpp::ClientBase::SharedPtr c): ptr(c) {};
  TaggedExecutable(rclcpp::Waitable::SharedPtr w): ptr(w) {};
  TaggedExecutable(TaggedExecutable const&) = default;
  ~TaggedExecutable() = default;
};

struct AnyExecutable
{
  RCLCPP_PUBLIC
  AnyExecutable();

  RCLCPP_PUBLIC
  virtual ~AnyExecutable();

  // Convenience functions (Checking union type)
  bool is_empty() {
    // This should be safe
    return _executable.ptr.empty.type == rclcpp::ExType::EmptyExec;
  }

  bool is_subscription() {
    // This should be safe
    return _executable.ptr.subscription.type == rclcpp::ExType::SubscriptionExec;
  }

  bool is_timer() {
    // This should be safe
    return _executable.ptr.timer.type == rclcpp::ExType::TimerExec;
  }

  bool is_service() {
    // This should be safe
    return _executable.ptr.service.type == rclcpp::ExType::ServiceExec;
  }

  bool is_client() {
    // This should be safe
    return _executable.ptr.client.type == rclcpp::ExType::ClientExec;
  }

  bool is_waitable() {
    // This should be safe
    return _executable.ptr.waitable.type == rclcpp::ExType::WaitExec;
  }

  // Convenience functions (extracting union data)
  rclcpp::SubscriptionBase::SharedPtr get_subscription() {
    return _executable.ptr.subscription.ptr;
  }

  rclcpp::TimerBase::SharedPtr get_timer() {
    return _executable.ptr.timer.ptr;
  }

  rclcpp::ServiceBase::SharedPtr get_service() {
    return _executable.ptr.service.ptr;
  }

  rclcpp::ClientBase::SharedPtr get_client() {
    return _executable.ptr.client.ptr;
  }

  rclcpp::Waitable::SharedPtr get_waitable() {
    return _executable.ptr.waitable.ptr;
  }

  // Convenience functions (setting union)
  void set_executable(rclcpp::SubscriptionBase::SharedPtr ptr) {
    _executable = TaggedExecutable(ptr);
  }

  // These are used to keep the scope on the containing items
  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  std::shared_ptr<void> data;

  private:
  // Only one of the following pointers will be set.
  TaggedExecutable _executable;
};

namespace executor
{

using AnyExecutable [[deprecated("use rclcpp::AnyExecutable instead")]] = AnyExecutable;

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__ANY_EXECUTABLE_HPP_

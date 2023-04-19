/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <memory>
#include <unordered_map>

#include "modules/common/util/factory.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class TaskFactory { // 用于实现工厂模式
 public:
  // task的创建由Init()静态函数完成，主要实现决策器decider、优化器optimizer等对象的创建与管理
  static void Init(const PlanningConfig &config,
                   const std::shared_ptr<DependencyInjector> &injector);
  static std::unique_ptr<Task> CreateTask(
      const TaskConfig &task_config,
      const std::shared_ptr<DependencyInjector> &injector);

 private:
  static apollo::common::util::Factory<
      TaskConfig::TaskType, Task,
      Task *(*)(const TaskConfig &config,
                const std::shared_ptr<DependencyInjector> &injector),
      std::unordered_map<
          TaskConfig::TaskType,
          Task *(*)(const TaskConfig &config,
                    const std::shared_ptr<DependencyInjector> &injector),
          std::hash<int>>>
      task_factory_; // 用于储存各类task
  static std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
      default_task_configs_;
};

}  // namespace planning
}  // namespace apollo

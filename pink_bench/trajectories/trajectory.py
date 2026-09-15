#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import abc

from meshcat import Visualizer
from pink import Configuration, Task


class Trajectory:
    __metaclass__ = abc.ABCMeta

    def __init__(self, task: Task):
        self.task = task

    def reset(self, configuration: Configuration, viewer: Visualizer):
        self.task.set_target_from_configuration(configuration)

    @abc.abstractmethod
    def step(self, dt: float):
        pass

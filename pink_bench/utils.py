#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np


class RunningMeanStd:
    """Running average and standard deviation estimator."""

    def __init__(self):
        """Initialize estimator."""
        self.reset()

    def reset(self):
        """Reset estimator."""
        self.n = 0
        self.x = 0.0
        self.x2 = 0.0

    def append(self, x):
        self.n += 1
        self.x += x
        self.x2 += x**2

    def average(self):
        assert self.n >= 1
        return self.x / self.n

    def std(self):
        assert self.n > 1
        mu = self.average()
        unbiased = np.sqrt(1.0 + 1.0 / (self.n - 1))
        return unbiased * np.sqrt(self.x2 / self.n - mu**2)

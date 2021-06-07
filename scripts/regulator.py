"""@package regulator
Implements PID regulation algorithms
"""
#!/usr/bin/env python

class Regulator():
  def __init__(self, KP, KI, T, limit):
    self.KP = KP
    self.KI = KI
    self.limit = limit
    self.err_prev = 0.0
    self.u = 0.0
    self.T = T

  def update_params(self, KP, KI):
    self.KP = KP
    self.KI = KI

  def pid_positional(self, error):

    # Calculate control
    du = self.KP * (error - self.err_prev) + self.KI * self.T * error
    self.u = self.u + du

    # Anti wind-up
    if self.u > self.limit:
      self.u = self.limit
    elif self.u < -self.limit:
      self.u = -self.limit

    self.err_prev = error

    return self.u

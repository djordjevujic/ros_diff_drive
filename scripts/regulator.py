"""@package regulator
Implements PID regulation algorithms
"""
#!/usr/bin/env python

class Regulator():
  def __init__(self, KP, TI, TD, T, u_limit, ui_limit):
    self.KP = KP
    self.KI = KP / TI
    self.KD = KP * TD
    self.u_limit = u_limit
    self.ui_limit = ui_limit
    self.err_prev = 0.0
    self.err_p_prev = 0.0
    self.u = 0.0
    self.T = T
    self.KDT = self.KD / T
    self.KIT = self.KI * T

  # Note: Should be used during debugging and PID setup
  #       Not intended to be used during normal operation
  #       because of the runtime consumption
  def update_params(self, KP, TI, TD, ui_limit):
    self.KP = KP
    self.KI = KP / TI
    self.KD = KP * TD
    self.KDT = self.KD / self.T
    self.KIT = self.KI * self.T
    self.ui_limit = ui_limit

  def pid_positional(self, error):
    # Calculate separate outputs
    Up = self.KP * error
    Ui = self.Ui + self.KIT * error
    Ud = self.KDT * (error - self.err_prev)

    # Anti wind-up of integral output
    if Ui > self.ui_limit:
      Ui = self.ui_limit
    elif Ui < -self.ui_limit:
      Ui = -self.ui_limit

    # Backup variables needed for the next iteration
    self.Ui = Ui
    self.err_prev = error

    # Calculate overall output
    U = Up + Ui + Ud

    return U

  def pid_incremental(self, error):
    # Calculate control
    dUp = self.KP * (error - self.err_prev)
    dUi = self.KI * self.T * error
    dUd = self.KDT * (error - 2*self.err_prev + self.err_p_prev)
    self.u = self.u + dUp + dUi + dUd

    # Anti wind-up
    if self.u > self.u_limit:
      self.u = self.u_limit
    elif self.u < -self.u_limit:
      self.u = -self.u_limit

    # Backup values needed for the next iteration
    self.error_p_prev = self.err_prev
    self.err_prev = error

    return self.u

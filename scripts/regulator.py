"""
    Implements PID regulation algorithms
""" 
#!/usr/bin/env python

## Class of the regulator which contains parameters and methods which implement different control algorithms
class Regulator():
  ## Constructor of the regulator
  def __init__(self, KP, TI, TD, T, u_limit, ui_limit):
    ## KP Gain of the PID
    self.KP = KP

    ## KI Gain of the PID
    self.KI = KP / TI

    ## KD Gain of the PID
    self.KD = KP * TD

    ## Limit of the overall control output (only for incremental PID)
    self.u_limit = u_limit

    ## Limit of the integral controll output (only for positional PID)
    self.ui_limit = ui_limit

    ## Error from the previous interation
    self.err_prev = 0.0

    ## Error from the iteration before previous one
    self.err_p_prev = 0.0

    ## Calculated control output
    self.u = 0.0

    ## Period between two iteration
    self.T = T

    ## KD * T part of the PID calculation
    self.KDT = self.KD / T

    ## KI * T part of the PID calculation
    self.KIT = self.KI * T

  ## Method for updating PID parameters.
  ## Note: Should be used during debugging and PID setup
  ##       Not intended to be used during normal operation
  ##       because of the runtime consumption.
  def update_params(self, KP, TI, TD, ui_limit):
    self.KP = KP
    self.KI = KP / TI
    self.KD = KP * TD
    self.KDT = self.KD / self.T
    self.KIT = self.KI * self.T
    self.ui_limit = ui_limit

  ## Positional PID algorithm method
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
    ## Backup integral output control value
    self.Ui = Ui
    self.err_prev = error

    # Calculate overall output
    U = Up + Ui + Ud

    return U

  ## Incremental PID algorithm method
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
    ## Backup 
    self.err_p_prev = self.err_prev
    self.err_prev = error

    return self.u

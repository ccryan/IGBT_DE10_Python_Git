"""
Feedback Controller Class

Created: 10 June 2020
Updated: 13 July 2020
Verification: 14 July 2020

@author: Roland Probst and Anjana Heva

Revision 004:
- Done: added autotuning of magnets using sigmoid curve -AH 
- Done. output_enabled parameter (False by default): allows to enable/disable controller output during runtime. 
- Done. proportional_on_input parameter: proportional action on process variable eliminates the instant and possibly very large change in output caused by a sudden change to the setpoint.
- Testing. smoothing of measurement noise. Calculates the mean of past measurements as the input to the controller

"""

import numpy as np

class FeedbackController(object):
  """ 
  A simple feedback controller for controlling magnetization of electropermanent magnets. 
  The controller continuously calculates an error value as the difference between 
  a desired setpoint and a measured process variable and applies a correction based 
  on proportional, integral, and derivative terms.  
  """

  def __init__(
      self,
      Kp=0.0,
      Ki=0.0,
      Kd=0.0,
      beta=1,
      setpoint=0,
      output_limits=(None, None),
      setpoint_weighting=True,
      proportional_on_input=False,
      data_collection=True,
      sample_time=1,
      output_enabled=False,
      measurement_smoothing_enabled = False,
      measurement_smoothing_start = 10,
      measurement_smoothing_past = 5,
  ):
    """
    Initialize a new feedback controller.
    
    :param Kp: The value for the proportional gain Kp
    :param Ki: The value for the integral gain Ki
    :param Kd: The value for the derivative gain Kd
    :param beta: The value (between 0 and 1) for setpoint weighting.
    :param setpoint: The initial setpoint that the feedback controller will try to achieve 
    :param output_limits: The initial output limits to use, given as an iterable with 2
      elements, for example: (lower, upper). The output will never go below the lower limit
      or above the upper limit. Either of the limits can also be set to None to have no limit
      in that direction. Setting output limits also avoids integral windup, since the
      integral term will never be allowed to grow outside of the limits.
    :param setpoint_weighting: Setpoint weighting adds adjustable factors (usually between 0 and 1) 
      to the setpoint in the error in the proportional and derivative element of the controller. 
      The error in the integral term must be the true control error to avoid steady-state control errors. 
      These two extra parameters do not affect the response to load disturbances and measurement noise and 
      can be tuned to improve the controller's setpoint response.
    :param proportional_on_input: The proportional gain acts solely on the process variable. This means 
      that only the integral action responds to changes in the setpoint. The modification to the algorithm 
      does not affect the way the controller responds to process disturbances. Basing proportional action 
      on process variable eliminates the instant and possibly very large change in output caused by a sudden 
      change to the setpoint. Depending on the process and tuning this may be beneficial to the response to 
      a setpoint step. 
    :output_enabled: output_enabled parameter allows to enable/disable controller output during runtime 
      (False by default).
    :measurement_smoothing_enabled: controller input is estimated based on past measurements 
    """
    self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
    self.beta = beta
    self.setpoint = setpoint
    self.setpoint_weighting = setpoint_weighting
    self.proportional_on_input = proportional_on_input
    self.sample_time = sample_time
    self._min_output, self._max_output = output_limits
    self.output_enabled=output_enabled
    self.measurement_smoothing_enabled = measurement_smoothing_enabled
    self.measurement_smoothing_start = measurement_smoothing_start
    self.measurement_smoothing_past = measurement_smoothing_past
    
    # Data Collection
    self.data_collection = data_collection
    self.time_total = 0
    self.input_data = []
    self.output_data = []
    self.error_data = []
    self.integral_data = []

    # counter for smoothing filter
    self.counter = 0

    self.reset()

  def clamp(self, value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif upper is not None and value > upper:
        return upper
    elif lower is not None and value < lower:
        return lower
    return value

 # Sigmoid function along which BH parameter is defined

  def BH_parameter(self, input):
        error = self.setpoint - input
        """
        p1 = 4.493e-6
        p2 = -.00035
        p3 = 0.00801
        p4 = 0.9378
        return p1*(error**3) + p2*(error**2) + p3*(error) + p4
        """
        return (1 / (1 + np.exp((-abs(error)+25)*.15)))
        #return (1 / (1 + np.exp(-abs(error)*.1))) - 0.5


  def update(self, input):
    """
    Update the feedback controller. Call the feedback controller.update() with *input_* 
    to calculate and return a control output.
    """
    BH_factor = self.BH_parameter(input)
    
    # measurement smoothing
    if self.measurement_smoothing_enabled and self.counter > self.measurement_smoothing_start:
      input_filtered = np.mean(self.input_data[self.counter-self.measurement_smoothing_past:self.counter])
      input = input_filtered
    self.counter += 1

    # compute error term
    error = self.setpoint - input
    d_input = input - (self._last_input if self._last_input is not None else input)

    # compute the proportional term
    if self.setpoint_weighting:
      # compute proportional on fraction of setpoint
      self._proportional = self.Kp * (self.beta * self.setpoint - input)*BH_factor
    elif self.proportional_on_input:
      # add the proportional error on measurement to error_sum
      self._proportional -= self.Kp * d_input*BH_factor
    else:
      # compute proportional on full error
      self._proportional = self.Kp * error*BH_factor
    
    # compute intergral and derivative terms. 
    dt = self.sample_time

    self._integral += self.Ki * error * dt
    self._integral = self.clamp(self._integral, self.output_limits)  # avoid integral windup

    self._derivative = -self.Kd * d_input / dt

    # compute final output
    if self.output_enabled:
      output = self._proportional + self._integral + self._derivative
      output = self.clamp(output, self.output_limits)
    else:
      output = 0

    # keep track of state
    self._last_output = output
    self._last_input = input
    
    # collect data
    if self.data_collection:
      self.time_total += self.sample_time
      self.input_data.append(input)
      self.output_data.append(output)
      self.error_data.append(error)
      self.integral_data.append(self._integral)

    return output

  @property
  def tunings(self):
      """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
      return self.Kp, self.Ki, self.Kd

  @tunings.setter
  def tunings(self, tunings):
    """Set feedback controller tunings."""
    self.Kp, self.Ki, self.Kd = tunings
    
  @property
  def output_limits(self):
      """
      The current output limits as a 2-tuple: (lower, upper).
      See also the *output_limts* parameter in :meth:`FeedbackController.__init__`.
      """
      return self._min_output, self._max_output

  @output_limits.setter
  def output_limits(self, limits):
      """Set the output limits."""
      if limits is None:
          self._min_output, self._max_output = None, None
          return

      min_output, max_output = limits

      if None not in limits and max_output < min_output:
          raise ValueError('lower limit must be less than upper limit')

      self._min_output = min_output
      self._max_output = max_output

      self._integral = self.clamp(self._integral, self.output_limits)
      self._last_output = self.clamp(self._last_output, self.output_limits)

  def reset(self):
    """
    Reset the feedback controller internals.
    This sets each term to 0 as well as clearing the integral, the last output and the last
    input (derivative calculation).
    """
    self._proportional = 0
    self._integral = 0
    self._derivative = 0

    self._last_output = None
    self._last_input = None
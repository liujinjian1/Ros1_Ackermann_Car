# -*- coding: utf-8 -*-

import time


class PID_Control:
	def __init__(self, P = 0.5, I = 0.0, D = 0.0):

		self.Kp = P
		self.Ki = I
		self.Kd = D

		self.error_now = 0.0
		self.error_last = 0.0
		self.error_last_last = 0.0
		self.Pid_out = 0.0

	def PID_get_data(self, date, min_date, max_date):
		if date < min_date:
			return min_date
		elif date > max_date:
			return max_date
		else:
			return date

	def PID_update(self, error):
		self.error_now = error

		Kpdata = self.Kp*(self.error_last)

		Kidata = self.Ki*(self.error_now)

		Kddata = self.Kd*(self.error_now - 2*self.error_last + self.error_last_last)

		self.Pid_out =  Kpdata + Kidata + Kddata
		print(self.Pid_out, Kpdata,  Kidata,  Kddata)
		self.error_last_last = self.error_last
		self.error_last = self.error_now
		
		return self.Pid_out



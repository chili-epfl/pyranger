"""
Odometry calculation for the Ranger, based on ROS differential_drive package.

    Copyright (C) 2012 Jon Stephan, 2014 Séverin Lemaignan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
import time
from math import sin, cos

TICKS_METER = 130000 # The number of wheel encoder ticks per meter of travel
#TICKS_TO_PID = 0.005124 # motor speed to send to go to 1 tick/sec

BASE_WIDTH = 0.3

ENCODER_MIN = -32768
ENCODER_MAX = 32768

ENCODER_LOW_WRAP = (ENCODER_MAX - ENCODER_MIN) * 0.3 + ENCODER_MIN
ENCODER_HIGH_WRAP = (ENCODER_MAX - ENCODER_MIN) * 0.7 + ENCODER_MIN


class Odom:

    def __init__(self):

        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = time.time()

    def update(self, l_enc, r_enc):

        #### Left wheel
        if (l_enc < ENCODER_LOW_WRAP and self.prev_lencoder > ENCODER_HIGH_WRAP):
            self.lmult = self.lmult + 1

        if (l_enc > ENCODER_HIGH_WRAP and self.prev_lencoder < ENCODER_LOW_WRAP):
            self.lmult = self.lmult - 1

        self.left = 1.0 * (l_enc + self.lmult * (ENCODER_MAX - ENCODER_MIN)) 
        self.prev_lencoder = l_enc

        #### Right wheel
        if (r_enc < ENCODER_LOW_WRAP and self.prev_rencoder > ENCODER_HIGH_WRAP):
            self.rmult = self.rmult + 1

        if (r_enc > ENCODER_HIGH_WRAP and self.prev_rencoder < ENCODER_LOW_WRAP):
            self.rmult = self.rmult - 1

        self.right = 1.0 * (r_enc + self.rmult * (ENCODER_MAX - ENCODER_MIN)) 
        self.prev_rencoder = r_enc

        #print("L: %s    R: %s" % (self.left, self.right))

    def get(self):
        """ Returns (x, y, theta, v, w) for the robot (in meters, seconds and radians).
        """
        now = time.time()

        elapsed = now - self.then
        self.then = now

        # calculate odometry
        if self.enc_left == None:
            d_left = 0
            d_right = 0
        else:
            d_left = (self.left - self.enc_left) / TICKS_METER
            d_right = (self.right - self.enc_right) / TICKS_METER
        self.enc_left = self.left
        self.enc_right = self.right
        
        # distance traveled is the average of the two wheels 
        d = ( d_left + d_right ) / 2
        # this approximation works (in radians) for small angles
        th = ( d_right - d_left ) / BASE_WIDTH
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed
        
            
        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
            self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
        if( th != 0):
            self.th = self.th + th

        return (self.x, self.y, self.th, self.dx, self.dr)

    def twist_to_motors(self, v, w):
        """ Returns the speed to apply on left and right motors for a 
        given (v, w) (in m.s^-1 and rad.s^-1).

        Uses an approximation that works for small rotation speeds.

        Returns (left speed, right speed) in m.s^-1
        """
        # dx = (l + r) / 2
        # dr = (r - l) / w

        right = 1.0 * v + w * BASE_WIDTH / 2 
        left = 1.0 * v - w * BASE_WIDTH / 2

        return (left, right)

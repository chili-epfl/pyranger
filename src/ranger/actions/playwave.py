nosound = False
try:
    import alsaaudio

except ImportError:
    print("alsaaudio not available on your system. I won't play sounds!")
    nosound = True

import wave
import sys


class WavePlayer:
    def __init__(self):
        if nosound:
            return

        self.device = alsaaudio.PCM(card='default')

	self.device.setchannels(1) # mono
        self.device.setrate(16000)
        self.device.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self.device.setperiodsize(32)
        

    def play(self, filename):
        if nosound:
            return

        self.playing = True
        # open the file for reading.

	try:
            wf = wave.open(filename, 'rb')
        except IOError:
            print("Could not open sound <%s>!" % filename)
            return

        try:
            data = wf.readframes(1600)
            while self.playing and data:
        	    self.device.write(data)
        	    data = wf.readframes(1600)
	finally:
	    wf.close()
  
	 
    def stop(self):
        if nosound:
            return

        self.playing = False

if __name__ == "__main__":

    # validation. If a wave file hasn't been specified, exit.
    if len(sys.argv) < 2:
        print "Plays a wave file.\n\n" +\
            "Usage: %s filename.wav" % sys.argv[0]
        sys.exit(-1)

    player = WavePlayer()
    player.play(sys.argv[1])

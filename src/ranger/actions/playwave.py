nosound = False
try:
    import pyaudio
except ImportError:
    print("pyaudio, the Python port of the PortAudio library, is not available on your system. I won't play sounds!")
    nosound = True
import wave
import sys


class WavePlayer:
    def __init__(self):
        if nosound:
            return

        self.p = pyaudio.PyAudio()
        self.stream = None

    def play(self, filename):
        if nosound:
            return

        self.playing = True
        # length of data to read.
        chunk = 1024
        '''
        ************************************************************************
            This is the start of the "minimum needed to read a wave"
        ************************************************************************
        '''
        # open the file for reading.
        try:
            wf = wave.open(filename, 'rb')
        except IOError:
            print("Could not open sound <%s>!" % filename)
            return
        # create an audio object

        # open stream based on the wave object which has been input.
        self.stream = self.p.open(format = self.p.get_format_from_width(wf.getsampwidth()),
                        channels = wf.getnchannels(),
                        rate = wf.getframerate(),
                        output = True)

        # read data (based on the chunk size)
        data = wf.readframes(chunk)

        # play stream (looping from beginning of file to the end)
        while data != '' and self.playing:
            # writing to the stream is what *actually* plays the sound.
            self.stream.write(data)
            data = wf.readframes(chunk)

        # cleanup stuff.
        self.stream.close()    

    def stop(self):
        if nosound:
            return

        self.playing = False
        if self.stream:
            self.stream.close()

    def close(self):
        if nosound:
            return

        self.playing = False
        self.p.terminate()


if __name__ == "__main__":

    # validation. If a wave file hasn't been specified, exit.
    if len(sys.argv) < 2:
        print "Plays a wave file.\n\n" +\
            "Usage: %s filename.wav" % sys.argv[0]
        sys.exit(-1)

    player = WavePlayer()
    player.play(sys.argv[1])
    player.close()

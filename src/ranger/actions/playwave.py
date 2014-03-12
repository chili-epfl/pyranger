import pyaudio
import wave
import sys


p = pyaudio.PyAudio()

def playwave(filename):

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
    stream = p.open(format = p.get_format_from_width(wf.getsampwidth()),
                    channels = wf.getnchannels(),
                    rate = wf.getframerate(),
                    output = True)

    # read data (based on the chunk size)
    data = wf.readframes(chunk)

    # play stream (looping from beginning of file to the end)
    while data != '':
        # writing to the stream is what *actually* plays the sound.
        stream.write(data)
        data = wf.readframes(chunk)

    # cleanup stuff.
    stream.close()    

def close_audio():
    p.terminate()


if __name__ == "__main__":

    # validation. If a wave file hasn't been specified, exit.
    if len(sys.argv) < 2:
        print "Plays a wave file.\n\n" +\
            "Usage: %s filename.wav" % sys.argv[0]
        sys.exit(-1)

    playwave(sys.argv[1])
    close_audio()

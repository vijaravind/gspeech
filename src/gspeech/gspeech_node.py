#!/usr/bin/env python

# See https://github.com/achuwilson/gspeech for original work.


import json, os, shlex, socket, subprocess, sys
import pyaudio, wave
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import String
from gspeech.msg import Speech


class GSpeech(object):
    """Speech Recogniser using Google Speech API"""

    def __init__(self, _api_key):
        """Constructor"""
        # pyaudio setup
        self.pa_channels = 1
        self.pa_buf_len = 1024
        self.pa_format = pyaudio.paInt16
        self.pa_rate = 44100
        self.pa_wave_out_file = '/tmp/recording.wav'
        self.pa_flac_out_file = '/tmp/recording.flac'
        self.pa_handle = pyaudio.PyAudio()
        self.pa_in_device_info = self.pa_handle.get_default_input_device_info()
        self.pa_stream = self.pa_handle.open(
            channels=self.pa_channels, frames_per_buffer=self.pa_buf_len,
            format=self.pa_format, input=True,
            input_device_index=self.pa_in_device_info['index'],
            rate=self.pa_rate, start=False
        )
        self.audio_frames = list()    # to store audio capture frames
        # configure system commands
        self.api_key = _api_key
        self.flac_convert = "flac -f {wav_file} -o {flac_file}".format(
            wav_file=self.pa_wave_out_file, flac_file=self.pa_flac_out_file
        )
        self.wget_cmd = ("wget -q -U \"Mozilla/5.0\" ") + \
            ("--post-file {flac_file} ") + \
            ("--header=\"Content-Type: audio/x-flac; rate=44100\" -O - ") + \
            ("\"https://www.google.com/speech-api/v2/recognize") + \
            ("?output=json&lang=en-us&key={api_key}\"")
        self.wget_cmd = self.wget_cmd.format(
            api_key=self.api_key, flac_file=self.pa_flac_out_file
        )
        self.wget_args = shlex.split(self.wget_cmd)
        # start ROS node
        rospy.init_node('gspeech', log_level=rospy.DEBUG)
        # configure ROS settings
        rospy.on_shutdown(self.shutdown)
        self.pub_confidence = rospy.Publisher('~confidence', Int8, queue_size=1)
        self.pub_jsonstr = rospy.Publisher('~jsonstr', String, queue_size=1)
        self.pub_recognising = rospy.Publisher('~recognising', Bool, queue_size=1)
        self.pub_speech = rospy.Publisher('~speech', Speech, queue_size=1)
        self.pub_transcript = rospy.Publisher('~transcript', String, queue_size=1)
        self.sub_start = rospy.Subscriber('~start', Bool, self.start_callback)
        self.sub_stop = rospy.Subscriber('~stop', Bool, self.stop_callback)
        # run speech recognition
        self.started = True
        self.recognising = False
        self.do_recognition()

    def start_callback(self, msg):
        """Start speech recognition"""
        if msg.data:
            if not self.started:
                self.started = True
                if not self.recognising:
                    self.do_recognition()
                rospy.loginfo("gspeech recogniser started")
            else:
                rospy.loginfo("gspeech is already running")

    def stop_callback(self, msg):
        """Stop speech recognition"""
        if msg.data:
            if self.started:
                self.started = False
                rospy.loginfo("gspeech recogniser stopped")
            else:
                rospy.loginfo("gspeech is already stopped")

    def shutdown(self):
        """Stop all system process before killing node"""
        self.started = False
        self.sub_start.unregister()
        self.sub_stop.unregister()
        self.pa_stream.close()
        self.pa_handle.terminate()

    def do_recognition(self):
        """Do speech recognition"""
        del self.audio_frames[:]    # empty the list
        self.pa_stream.start_stream()
        rospy.logdebug("recording started")
        while self.started:
            self.recognising = True
            self.pub_recognising.publish(Bool(self.recognising))
            # read audio
            self.audio_frames.append(self.pa_stream.read(self.pa_buf_len))

        self.pa_stream.stop_stream()
        rospy.logdebug("recording stopped")
        # save speech
        self.save_speech()
        # send audio data to google
        wget_out, wget_err = self.send_speech()
        # deal with response
        if not wget_err and len(wget_out) > 16:
            wget_out = wget_out.split('\n', 1)[1]
            a = json.loads(wget_out)['result'][0]
            transcript = ""
            confidence = 0
            if 'confidence' in a['alternative'][0]:
                confidence = a['alternative'][0]['confidence']
                confidence = confidence * 100
                self.pub_confidence.publish(confidence)
                rospy.loginfo("confidence: {}".format(confidence))
            if 'transcript' in a['alternative'][0]:
                transcript = a['alternative'][0]['transcript']
                self.pub_transcript.publish(String(transcript))
                rospy.loginfo("transcript: {}".format(data))
            self.pub_speech.publish(
                Speech(transcript=transcript, confidence=confidence)
            )
            self.pub_jsonstr.publish(String(wget_out))

        self.recognising = False
        self.pub_recognising.publish(Bool(self.recognising))
        self.pa_stream.stop_stream()

    def save_speech(self):
        """Save first to WAVE file in `/tmp` folder and convert FLAC format"""
        rospy.logdebug("saving wav file")
        wav_file = wave.open(self.pa_wave_out_file, 'wb')
        wav_file.setnchannels(self.pa_channels)
        wav_file.setsampwidth(self.pa_handle.get_sample_size(self.pa_format))
        wav_file.setframerate(self.pa_rate)
        wav_file.writeframes(b''.join(self.audio_frames))
        wav_file.close()
        rospy.logdebug("converting to flac file")
        os.system(self.flac_convert)

    def send_speech(self):
        """Send speech (FLAC format) to Google servers and return response"""
        rospy.logdebug("sending speech")
        wget_out, wget_err = subprocess.Popen(
            self.wget_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        ).communicate()
        return wget_out, wget_err


def is_connected():
    """Check if connected to Internet"""
    try:
        # check if DNS can resolve hostname
        remote_host = socket.gethostbyname("www.google.com")
        # check if host is reachable
        s = socket.create_connection(address=(remote_host, 80), timeout=5)
        return True
    except:
        pass
    return False


def usage():
    """Print Usage"""
    print("Usage:")
    print("rosrun gspeech gspeech.py <API_KEY>")


def main():
    if len(sys.argv) < 2:
        usage()
        sys.exit("No API_KEY provided")
    if not is_connected():
        sys.exit("No Internet connection available")
    api_key = str(sys.argv[1])
    speech = GSpeech(api_key)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sys.exit(0)



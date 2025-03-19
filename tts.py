from pyt2s.services import stream_elements
from pydub import AudioSegment
from io import BytesIO
import simpleaudio as sa

data = stream_elements.requestTTS('This is a test of a text to speech library.', stream_elements.Voice.Joanna.value)

audio = AudioSegment.from_mp3(BytesIO(data))

play_obj = sa.play_buffer(audio.raw_data, num_channels=audio.channels, bytes_per_sample=audio.sample_width, sample_rate=audio.frame_rate)
play_obj.wait_done()

#Nicole #british 
#Emma #baby-like
#Amy #i like this 
#Joanna #little 
#Kendra #slower and clearer 
#Kimberly #also ok 
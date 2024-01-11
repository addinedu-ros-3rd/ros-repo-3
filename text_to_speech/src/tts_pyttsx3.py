
# https://github.com/nateshmbhat/pyttsx3
# pip install pyttsx3
# sudo apt update && sudo apt install espeak ffmpeg libespeak1

####################Example#####################
# import pyttsx3
# engine = pyttsx3.init()
# engine.say("I will speak this text")
# engine.runAndWait()
################################################

##########Change voice, Rate and Volume########
import pyttsx3
engine = pyttsx3.init() # object creation

""" RATE"""
rate = engine.getProperty('rate')   # getting details of current speaking rate
print (rate)                        #printing current voice rate
engine.setProperty('rate', 100)     # setting up new voice rate


"""VOLUME"""
volume = engine.getProperty('volume')   #getting to know current volume level (min=0 and max=1)
print (volume)                          #printing current volume level
engine.setProperty('volume',1.0)    # setting up volume level  between 0 and 1

"""VOICE"""
voices = engine.getProperty('voices')       #getting details of current voice
#engine.setProperty('voice', voices[0].id)  #changing index, changes voices. o for male
engine.setProperty('voice', voices[1].id)   #changing index, changes voices. 1 for female

engine.say('Hello 안녕하세요 이것은 pyttsx3의 text to speach 입니다 감사합니다')
# engine.say('My current speaking rate is ' + str(rate))
engine.runAndWait()
engine.stop()

"""Saving Voice to a file"""
# On linux make sure that 'espeak' and 'ffmpeg' are installed
engine.save_to_file('Hello 안녕하세요 이것은 pyttsx3의 text to speach 입니다 감사합니다', './data/tts_pyttsx3.mp3')
engine.runAndWait()
#pip install gTTS 

##############################Example##############################
# from gtts import gTTS
# text ="Hello 안녕하세요 이것은 Goolge text to speach 입니다 감사합니다"

# tts = gTTS(text=text, lang='ko')  #'en' Woman / 'ko' Man
# tts.save("tts_gTTS.mp3")
###################################################################

############################Save and Play##########################
#pip install gTTS pygame

from gtts import gTTS
import pygame

# 텍스트를 음성으로 변환하고 파일로 저장
text = "Hello 안녕하세요 이것은 Google text to speech 입니다 감사합니다"
tts = gTTS(text=text, lang='ko')  # 'ko'를 사용하여 한국어 음성 생성
tts.save("./data/tts_gTTS_ko.mp3")

# pygame 초기화
pygame.mixer.init()

# 음성 파일 로드
pygame.mixer.music.load("./data/tts_gTTS_ko.mp3")

# 재생
pygame.mixer.music.play()

# 음성 재생이 끝날 때까지 대기
while pygame.mixer.music.get_busy():
    pygame.time.Clock().tick(10)
####################################################################
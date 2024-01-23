const startButton = document.getElementById('startButton');
const stopButton = document.getElementById('stopButton');
const message = document.getElementById('message');
const userInput = document.getElementById('userInput');
const askButton = document.getElementById('askButton');
const soragodongResponse = document.getElementById('soragodongResponse');


let recognition = new webkitSpeechRecognition() || new SpeechRecognition();
recognition.lang = 'ko-KR'; // 한국어 설정
recognition.interimResults = false; // 중간 결과 무시
recognition.maxAlternatives = 1; // 대안의 최대 개수

startButton.addEventListener('click', function() {
    recognition.start();
    soragodongResponse.textContent = '';
});

stopButton.addEventListener('click', function() {
    recognition.stop();
});

recognition.onstart = function() {
    startButton.disabled = true;
    stopButton.disabled = false;
    message.textContent = '음성을 인식 중입니다...';
};

recognition.onresult = function(event) {
    const last = event.results.length - 1;
    const text = event.results[last][0].transcript;
    message.textContent = '인식된 메시지: ' + text;
    userInput.value = text;
    // 여기에서 특정 텍스트에 대한 로직을 추가할 수 있습니다.
};

recognition.onend = function() {
    startButton.disabled = false;
    stopButton.disabled = true;

    // 음성 인식이 끝나면 자동으로 '묻기' 버튼의 기능을 실행
    askButton.click();
};

const responses = ['가능해', '안돼', '진행시켜', '빨리해', '허락한다', '다시 한번 질문해', '좀 곤란한 걸?', '추천하지 않아'];

askButton.addEventListener('click', function() {
    const userInputText = userInput.value.trim(); // 앞뒤 공백 제거, 실수로 공백만 입력 방지

    // 문자열이 들어있다면 참, 없다면 거짓
    if (userInputText) { 
        const randomResponse = responses[Math.floor(Math.random() * responses.length)];
        soragodongResponse.textContent = '소라고동: ' + randomResponse;
    } else {
        soragodongResponse.textContent = '다시 물어봐';
    }

    stopButton.click();
    message.textContent = ''; // voice message 초기화
    userInput.value = ''; // userInput 초기화
});
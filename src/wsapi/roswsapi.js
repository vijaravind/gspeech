// flag variable to track state of recogniser
var recognising = false;
// setup ROS
var wsapi_ros = new ROSLIB.Ros({
    url: ROS_BRIDGE_URI
})
wsapi_ros.on('connection', function() {
    console.log('Connected to websocket server.');
});
wsapi_ros.on('error', function(error) {
    console.log('Websocket server connection error: ', error);
});
wsapi_ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});
// publishers
var str_dummy = new ROSLIB.Message({
    data: ''
});
var bool_dummy = new ROSLIB.Message({
    data: false
});
var float_dummy = new ROSLIB.Message({
    data: 0.0
})
var transcript_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/transcript',
    messageType: 'std_msgs/String'
});
var transcript_onend_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/transcript_onend',
    messageType: 'std_msgs/String'
});
var transcript_interim_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/transcript_interim',
    messageType: 'std_msgs/String'
});
var error_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/error',
    messageType: 'std_msgs/String'
});
var speech_start_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/speech_start',
    messageType: 'std_msgs/Bool'
});
var speech_end_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/speech_end',
    messageType: 'std_msgs/Bool'
});
var sound_start_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/sound_start',
    messageType: 'std_msgs/Bool'
});
var sound_end_pub = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: 'wsapi/sound_end',
    messageType: 'std_msgs/Bool'
});
//var confidence_pub = new ROSLIB.Topic({
//  ros: wsapi_ros,
//  name: 'wsapi/confidence',
//  messageType: 'std_msgs/Float32'
//});
//var confidence_interim_pub = new ROSLIB.Topic({
//  ros: wsapi_ros,
//  name: 'wsapi/confidence_interim',
//  messageType: 'std_msgs/Float32'
//});
// init publishers
transcript_pub.publish(str_dummy);
transcript_onend_pub.publish(str_dummy);
transcript_interim_pub.publish(str_dummy);
error_pub.publish(str_dummy);
speech_start_pub.publish(bool_dummy);
speech_end_pub.publish(bool_dummy);
sound_start_pub.publish(bool_dummy);
sound_end_pub.publish(bool_dummy);
//confidence_pub.publish(float_dummy);
//confidence_interim_pub.publish(float_dummy);
// subscribers
var start_listener = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: '/wsapi/start',
    messageType: 'std_msgs/Bool'
});
var stop_listener = new ROSLIB.Topic({
    ros: wsapi_ros,
    name: '/wsapi/stop',
    messageType: 'std_msgs/Bool'
});
// subscriber callback methods
start_listener.subscribe(function(msg) {
    console.log('Received message on /wsapi/start');
    if(!recognising && msg.data) {
        toggleStartStop();
    }
});
stop_listener.subscribe(function(msg) {
    console.log('Received message on /wsapi/stop');
    console.log('recognising: ' + recognising);
    console.log('data: ' + msg.data);
    if(recognising && msg.data) {
        toggleStartStop();
    }
});
// speech Recognition
var ignore_onend;
var final_transcript = '';
var start_timestamp;
var debug = true;
if(!debug) {
    dbg_span.style.display = 'none';
}
if(!('webkitSpeechRecognition' in window)) {
    window.alert('Upgrade browser!');
} else {
    var recognition = new webkitSpeechRecognition();
    recognition.continuous = true;
    recognition.interimResults = true;
    // on start
    recognition.onstart = function() {
        recognising = true;
        mic_img.src = 'mic-animate.gif';
        dbg_span.innerHTML = 'Start speaking...';
    };
    // on end
    recognition.onend = function() {
        console.log('recongnition.onend() called')
        recognising = false;
        // publish result
        var msg = new ROSLIB.Message({
            data: final_transcript
        });
        transcript_onend_pub.publish(msg);
        if(ignore_onend) {
            return;
        }
        mic_img.src = 'mic.gif';
        dbg_span.innerHTML = 'Stopped...';
    };
    // on error
    recognition.onerror = function(event) {
        var msg = new ROSLIB.Message({
            data: event.error
        });
        error_pub.publish(msg);
        mic_img.src = 'mic.gif';
        ignore_onend = true;
        dbg_span.innerHTML = 'Error: ' + event.error;
    };
    // on result
    recognition.onresult = function(event) {
        if(typeof(event.results) == 'undefined') {
            recognition.onend = null;
            recognition.stop();
            dbg_span.innerHTML == 'Undefined result type'
                return;
        }
        var interim_transcript = '';
        var confidence = 0.0;
        var msg;
        for(var idx = event.resultIndex; idx < event.results.length; ++idx) {
            if(event.results[idx].isFinal) {
                final_transcript += event.results[idx][0].transcript;
                confidence = event.results[idx][0].confidence;
                // publish
                msg = new ROSLIB.Message({
                    data: final_transcript
                });
                transcript_pub.publish(msg);
            } else {
                interim_transcript += event.results[idx][0].transcript;
                confidence = event.results[idx][0].confidence;
                // publish
                msg = new ROSLIB.Message({
                    data: interim_transcript
                });
                transcript_interim_pub.publish(msg);
            }
            final_span.innerHTML = final_transcript;
            interim_span.innerHTML = interim_transcript;
        }
    };
    // on speechstart
    recognition.onspeechstart = function(event) {
        var msg = new ROSLIB.Message({
            data: true
        });
        speech_start_pub.publish(msg);
    };
    // on speechend
    recognition.onspeechend = function(event) {
        var msg = new ROSLIB.Message({
            data: true
        });
        speech_end_pub.publish(msg);
    };
    // on soundstart
    recognition.onsoundstart = function(event) {
        var msg = new ROSLIB.Message({
            data: true
        });
        sound_start_pub.publish(msg);
    };
    // on soundend
    recognition.onsoundend = function(event) {
        var msg = new ROSLIB.Message({
            data: true
        });
        sound_end_pub.publish(msg);
    };
}
function toggleStartStop(event) {
    if(recognising) {
        console.log('calling recognition.stop()')
        recognition.stop();
        return;
    }
    final_transcript = '';
    recognition.start();
    final_span.innerHTML = '';
    mic_img.src = 'mic-slash.gif';
    //start_timestamp = event.timeStamp;
}

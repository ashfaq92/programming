// worker.js

// Function to handle messages sent from the main script
self.onmessage = function (e) {
    setTimeout(function() {
        pingPong(e)
    }, 3000)
}

function pingPong(event) {

    self.postMessage("you said " + event.data + " i say: PONG")
}

// Create a new web worker from the worker script
const worker = new Worker("worker.js")

// Define a function to handle messages from the worker
worker.onmessage = function (event) {
    // Update the result element with the data sent from the worker
    document.getElementById("result").textContent = event.data
}

// Send a message to the worker (in this example, the number 5)
worker.postMessage('PING')

const videos = {
    video1: "videos/demovideo1",
    video2: "videos/demovideo2",
}

let effectFunction = null

window.onload = function () {
    let controlLinks = document.querySelectorAll("a.control")
    for (let i = 0; i < controlLinks.length; i++) {
        controlLinks[i].onclick = handleControl
    }
    let effectLinks = document.querySelectorAll("a.effect")
    for (let i = 0; i < effectLinks.length; i++) {
        effectLinks[i].onclick = setEffect
    }
    let videoLinks = document.querySelectorAll("a.videoSelection")
    for (let i = 0; i < videoLinks.length; i++) {
        videoLinks[i].onclick = setVideo
    }
    // pushUnpushButtons("video1", [])
    // pushUnpushButtons("normal", [])
}

function setVideo(e) {
    playAudio()
    const id = e.target.getAttribute("id")
    const video = document.getElementById("video")
    if (id == "video1") {
        video.src = videos.video1 + getFormatExtension()
        pushUnpushButtons("video1", ["video2", "play", "pause", "loop", "mute"])
    } else if (id == "video2") {
        video.src = videos.video2 + getFormatExtension()
        pushUnpushButtons("video2", ["video1", "play", "pause", "loop", "mute"])
    }
    video.addEventListener("ended", endedHandler, false)
    video.addEventListener("play", processFrame, false)
}

function endedHandler() {
    pushUnpushButtons("", ["video1", "video2", "play", "mute", "loop"])
}

function handleControl(e) {
    playAudio()

    const id = e.target.getAttribute("id")
    const video = document.getElementById("video")
    switch (id) {
        case "play":
            pushUnpushButtons("play", ["pause"])
            if (video.ended) {
                video.load()
            }
            video.play()
            break
        case "pause":
            video.pause()
            pushUnpushButtons("pause", ["play"])
            break
        case "loop":
            if (isButtonPushed("loop")) {
                pushUnpushButtons("", ["loop"])
            } else {
                pushUnpushButtons("loop", [])
            }
            video.loop = !video.loop
            break
        case "mute":
            if (isButtonPushed("mute")) {
                pushUnpushButtons("", ["mute"])
            } else {
                pushUnpushButtons("mute", [])
            }
            video.muted = !video.muted
            break
        default:
            break
    }
}

function pushUnpushButtons(idToPush, idarrayToUnpush) {
    if (idToPush != "") {
        var anchor = document.getElementById(idToPush)
        var theClass = anchor.getAttribute("class")
        if (!theClass.indexOf("selected") >= 0) {
            theClass = theClass + " selected"
            anchor.setAttribute("class", theClass)
            var newImage = "url(images/" + idToPush + "pressed.png)"
            anchor.style.backgroundImage = newImage
        }
    }
    for (var i = 0; i < idarrayToUnpush.length; i++) {
        anchor = document.getElementById(idarrayToUnpush[i])
        theClass = anchor.getAttribute("class")
        if (theClass.indexOf("selected") >= 0) {
            theClass = theClass.replace("selected", "")
            anchor.setAttribute("class", theClass)
            anchor.style.backgroundImage = ""
        }
    }
}

function isButtonPushed(id) {
    let anchor = document.getElementById(id)
    let theClass = anchor.getAttribute("class")
    return theClass.indexOf("selected") >= 0
}

function setEffect(e) {
    playAudio()
    let id = e.target.getAttribute("id")
    if (id == "normal") {
        pushUnpushButtons("normal", ["western", "noir", "scifi"])
        effectFunction = null
    } else if (id == "western") {
        pushUnpushButtons("western", ["normal", "noir", "scifi"])
        effectFunction = western
    } else if (id == "noir") {
        pushUnpushButtons("noir", ["normal", "western", "scifi"])
        effectFunction = noir
    } else if (id == "scifi") {
        pushUnpushButtons("scifi", ["normal", "western", "noir"])
        effectFunction = scifi
    }
}

function processFrame() {
    checkCanvasSupport()
    const video = document.getElementById("video")
    // check if video is paused or
    if (video.ended || video.paused) {
        return
    }
    const bufferCanvas = document.getElementById("bufferCanvas")
    const displayCanvas = document.getElementById("displayCanvas")
    bufferCanvasCtx = bufferCanvas.getContext("2d")
    displayCanvasCtx = displayCanvas.getContext("2d")

    bufferCanvasCtx.drawImage(
        video,
        0,
        0,
        bufferCanvas.width,
        bufferCanvas.height
    )
    const frame = bufferCanvasCtx.getImageData(
        0,
        0,
        bufferCanvas.width,
        bufferCanvas.height
    )
    // console.log(frame);
    const length = frame.data.length / 4
    for (let i = 0; i < length; i++) {
        const r = frame.data[i * 4 + 0]
        const g = frame.data[i * 4 + 1]
        const b = frame.data[i * 4 + 2]
        if (effectFunction) {
            effectFunction(i, r, g, b, frame.data)
        }
    }
    displayCanvasCtx.putImageData(frame, 0, 0)
    setTimeout(processFrame, 0)
}

function noir(pos, r, g, b, data) {
    let brightness = (3 * r + 4 * g + b) >>> 3
    if (brightness < 0) brightness = 0
    data[pos * 4 + 0] = brightness
    data[pos * 4 + 1] = brightness
    data[pos * 4 + 2] = brightness
}

function bwcartoon(pos, r, g, b, outputData) {
    let offset = pos * 4
    if (outputData[offset] < 120) {
        outputData[offset] = 80
        outputData[++offset] = 80
        outputData[++offset] = 80
    } else {
        outputData[offset] = 255
        outputData[++offset] = 255
        outputData[++offset] = 255
    }
    outputData[++offset] = 255
    ++offset
}

function western(pos, r, g, b, data) {
    var brightness = (3 * r + 4 * g + b) >>> 3
    data[pos * 4 + 0] = brightness + 40
    data[pos * 4 + 1] = brightness + 20
    data[pos * 4 + 2] = brightness - 20
}
function scifi(pos, r, g, b, data) {
    var offset = pos * 4
    data[offset] = Math.round(255 - r)
    data[offset + 1] = Math.round(255 - g)
    data[offset + 2] = Math.round(255 - b)
}

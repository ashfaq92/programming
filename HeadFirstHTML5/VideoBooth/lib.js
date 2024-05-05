
function playAudio() {
    const url = "audio/old-radio-button-click.mp3"
    new Audio(url).play()
}

function getFormatExtension() {
    if (video.canPlayType("video/mp4") != "") {
        return ".mp4"
    } else if (video.canPlayType("video/webm") != "") {
        return ".webm"
    } else if (video.canPlayType("video/ogg") != "") {
        return ".ogg"
    } else {
        console.error("error in getting video format!")
    }
}

function checkCanvasSupport() {
    const canvas = document.createElement("canvas")
    if (!canvas.getContext) {
        const msg = "Please upgrade your browser to use canvas!"
        console.error(msg)
        alert(msg)
        return
    }
}
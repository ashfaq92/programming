let playlist = [
   'videos/preroll',
   'videos/areyoupopular',
   'videos/destinationearth'
]

let position = 0
let video


window.onload = function() {
    video = document.getElementById('video')
    loadVideo()
    video.addEventListener('ended', nextVideo, false)
}


function loadVideo() {
    video.src = `${playlist[position]}${getFormatExtension()}`
    console.log(video.src);
    video.play()
}

function nextVideo() {
    position++
    if (position >= playlist.length) {
        alert('show ended!')
        position = 0
    }
    loadVideo()
}


function getFormatExtension() {
    if (video.canPlayType('video/mp4') != '') {
        return '.mp4'
    } else if (video.canPlayType('video/webm') != '') {
        return '.webm'
    } else if (video.canPlayType('video/ogg') != '') {
        return '.ogg'
    } else {
        console.error('error in getting vidoe format!')
    }
}
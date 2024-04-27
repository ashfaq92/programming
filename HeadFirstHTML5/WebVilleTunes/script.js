window.onload = init

function init() {
    loadPlaylist()
    let addSongBtn = document.getElementById('addSongBtn')
    addSongBtn.onclick = addSong

}

function addSong() {
    let songName = document.getElementById('songName').value
    if (songName == "") {
        alert("Please enter a valid song name")
    } else {
        let li = document.createElement('li')
        li.classList.add('song')
        // li.innerHTML = songName
        li.appendChild(document.createTextNode(songName))
        // code to append li to existing uls
        let playlist = document.getElementById('playlist')
        playlist.appendChild(li)
        save(songName)
    }
}

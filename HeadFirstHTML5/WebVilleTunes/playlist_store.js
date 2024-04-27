function save(item) {
    let playlistarray = getStorearray("playlist")
    playlistarray.push(item)
    localStorage.setItem("playlist", JSON.stringify(playlistarray))
}

function loadPlaylist() {
    let playlistarray = getSavedSongs()
    let ul = document.getElementById("playlist")
    if (playlistarray != null) {
        for (let i = 0; i < playlistarray.length; i++) {
            let li = document.createElement("li")
            li.classList.add('song')
            li.appendChild(document.createTextNode(playlistarray[i]))
            ul.appendChild(li)
        }
    }
}

function getSavedSongs() {
    return getStorearray("playlist")
}

function getStorearray(key) {
    let playlistarray = localStorage.getItem(key)
    if (playlistarray == null || playlistarray == "") {
        playlistarray = new Array()
    } else {
        playlistarray = JSON.parse(playlistarray)
    }
    return playlistarray
}

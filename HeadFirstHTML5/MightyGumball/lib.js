const loadData = (url, type, callback) => {
    const xhr = new XMLHttpRequest()    // xhr = xml, http, request
    xhr.open(type, url)

    xhr.onload = () => {
        if (xhr.status == 200) {
            callback(xhr.responseText)
        } else {
            console.error(xhr)
        }
    }

    xhr.send(null)
}


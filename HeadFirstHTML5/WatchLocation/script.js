window.onload = () => init()

let watchId = null
let map = null
let prevCoords = null

const init = () => {
    const watchMeBtn = document.getElementById("watchMe")
    watchMeBtn.onclick = watchLocation

    const clearWatchBtn = document.getElementById("clearWatch")
    clearWatchBtn.onclick = clearWatch
}

const clearWatch = () => {
    if (watchId) {
        navigator.geolocation.clearWatch(watchId)
        watchId = null
        alert("location watch stopped.")
    } else {
        alert("no active location watch to stop")
    }
}

const watchLocation = () => {
    const options = {
        timeout: 3000,
        enableHighAccuracy: true,
        maximumAge: 0,
    }
    if (navigator.geolocation) {
        watchId = navigator.geolocation.watchPosition(
            displayLocation,
            displayLocationErr,
            options
        )
    } else {
        const errMsg = `Error: No geolocation support`
        const textNode = document.createTextNode(errMsg)
        const msgDiv = document.getElementById("msg")
        msgDiv.classList.remove("success-text")
        msgDiv.classList.add("error-text")
        msgDiv.replaceChildren(textNode)
    }
}

const displayLocationErr = (error) => {
    let errMsg = `${error.constructor.name} : ${error.message}, Code: ${error.code}`
    if (error.code == 3) {
        errMsg += " retrying..."
    }
    const textNode = document.createTextNode(errMsg)

    const msgDiv = document.getElementById("msg")
    msgDiv.classList.remove("success-text")
    msgDiv.classList.add("error-text")

    msgDiv.replaceChildren(textNode)
}

const displayLocation = (position) => {
    const msg = `Your Latitude is: ${position.coords.latitude} and longitude is:  ${position.coords.longitude}`
    const textNode = document.createTextNode(msg)

    const msgDiv = document.getElementById("msg")
    msgDiv.classList.remove("error-text")
    msgDiv.classList.add("success-text")
    msgDiv.replaceChildren(textNode)

    if (map == null) {
        showMap(position.coords)
        prevCoords = position.coords
    } else {
        let distanceFromPrevious  = computeDistance(position.coords, prevCoords) * 1000
        if (distanceFromPrevious  > 20) {
            scrollMapToPosition(position.coords)
            prevCoords = position.coords
        }
    }
}

const showMap = (coords) => {
    const centerCoords = new Microsoft.Maps.Location(
        coords.latitude,
        coords.longitude
    )
    let mapDiv = document.getElementById("myMap")
    const mapOptions = {
        center: centerCoords,
        zoom: 17,
    }
    map = new Microsoft.Maps.Map(mapDiv, mapOptions)

    let pushpin = new Microsoft.Maps.Pushpin(centerCoords, {
        // title: 'Your Location',
        // subTitle: 'This is your exact location'
    })
    map.entities.push(pushpin)

    const infoboxOptions = {
        title: "You are here",
        description: coords.latitude + " , " + coords.longitude,
        visible: false, // Set to false initially to hide the infobox
        offset: new Microsoft.Maps.Point(0, 20), // Offset the infobox from the pushpin
    }

    //  Create the infobox
    let infobox = new Microsoft.Maps.Infobox(centerCoords, infoboxOptions)
    map.entities.push(infobox)

    // Add click event to show the infobox when the pushpin is clicked
    Microsoft.Maps.Events.addHandler(pushpin, "click", function () {
        infobox.setOptions({ visible: true })
    })

    //TODO: Add navigation control  (directions)
    //TODO: Add directions functionality
    //TODO: Attach search and directions to infobox
}

const scrollMapToPosition = (coords) => {
    // stub test code
    // console.log("in", scrollMapToPosition.name)
    // let newLatitude = 62.240689
    // let newLongitude = 25.746698
    // const centerCoords = new Microsoft.Maps.Location(newLatitude, newLongitude)

    const centerCoords = new Microsoft.Maps.Location(
        coords.latitude,
        coords.longitude
    )
    map.setView({ center: centerCoords, zoom: map.getZoom() })

    let pushpin = new Microsoft.Maps.Pushpin(centerCoords, {
        // title: 'Your Location',
        // subTitle: 'This is your exact location'
    })
    map.entities.push(pushpin)
}

const computeDistance = (startCoords, endCoords) => {
    const startLatRads = degreeToRads(startCoords.lat)
    const startLonRads = degreeToRads(startCoords.lon)
    const endLatRads = degreeToRads(endCoords.lat)
    const endLonRads = degreeToRads(endCoords.lon)
    const radius = 6371
    const distance =
        Math.acos(
            Math.sin(startLatRads) * Math.sin(endLatRads) +
                Math.cos(startLatRads) *
                    Math.cos(endLatRads) *
                    Math.cos(startLonRads - endLonRads)
        ) * radius
    return distance
}


function degreeToRads(angle) {
    const radians = (angle * Math.PI) / 180
    return radians
}
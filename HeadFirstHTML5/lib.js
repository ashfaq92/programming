function getTimeFromString(timeString) {
    // Create a new Date object
    var theTime = new Date()

    // Use a regular expression to match the time string
    var time = timeString.match(/(\d+)(?::(\d\d))?\s*(p?)/)

    // Set the hours of the Date object
    theTime.setHours(parseInt(time[1]) + (time[3] ? 12 : 0))

    // Set the minutes of the Date object
    theTime.setMinutes(parseInt(time[2]) || 0)

    // Return the time in milliseconds
    return theTime.getTime()
}

module.exports = {
    getTimeFromString: getTimeFromString
}
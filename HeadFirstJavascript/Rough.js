function showClimateMsg() {
    //return;
    alert(constructMessage());
}

function constructClimateMsg() {
    var msg = "";
    msg += "Local "; // "Local ";
    if (getTemp() > 80)
        msg += "warming ";
    else
        msg += "cooling ";
    msg += "is ";
    if (getTemp() <= 70)
        return msg + "a hoax!";
    else
        return msg + "real!";
}

function getTemp() {
    // Read the actual temperature
    var actualTemp = readSensor();
    return actualTemp;
}

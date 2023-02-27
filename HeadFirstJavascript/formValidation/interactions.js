window.onload = (event) => {
    document.getElementById("error").style.display = "none"
    document.getElementById("output").style.display = "none"
}

function validateNumber(val, field) {
    let errorPara = document.getElementById("error")
    if (!isNumber(Number(val))) {
        errorPara.innerHTML = `Invalid ${field}`
        errorPara.style.display = "block"
    } else {
        errorPara.style.display = "none"
    }
}

function validateZipCode(val) {
    let errorPara = document.getElementById("error")
    if (!isZipCode(val)) {
        errorPara.innerHTML = `Invalid ZIP code`
        errorPara.style.display = "block"
    } else {
        errorPara.style.display = "none"
    } 
}

document.getElementById("shop").onclick = function findHouses(form) {
    let bedrooms = Number(document.getElementById("bedrooms").value)
    let zipCode = Number(document.getElementById("zipCode").value)    
    if (isNumber(bedrooms) && isZipCode(zipCode)) {
        console.log(bedrooms, zipCode)
        //form.submit()
    } else {
        console.error("Invalid bedrooms or zipCode value")
    }
    
}

function calculatePrice() {
    let income = Number(document.getElementById("income").value)
    let outputPara = document.getElementById("output")
    if (isNumber(income)) {
        let result = priceCalculator(income)
        outputPara.innerHTML = result
        outputPara.style.display = "block"
        outputPara.style.color = "green"
    } else {
        outputPara.innerHTML = "Invalid number"
        outputPara.style.display = "block"
        outputPara.style.color = "red"

    }
}



function isNumber(val) {
    return typeof val === 'number' && isFinite(val)
}



function isZipCode(val) {
    return /(^\d{5}$)|(^\d{5}-\d{4}$)/.test(val);
}



function priceCalculator(income) {
    return income * 4
}
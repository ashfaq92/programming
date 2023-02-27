function calculateSubtotal(cakeDonuts, glazedDonuts, pricePerDonut) {
    if ((Number(cakeDonuts) && Number(glazedDonuts) && Number(pricePerDonut) )) {
        return (cakeDonuts + glazedDonuts) * pricePerDonut
    } else {
        return console.error("invalid parameters passed to calculateSubtotal function")
    }
}

function calculateTax(subTotal, taxRate) {
    if ((Number(subTotal) && Number(taxRate))) {
        return subTotal * taxRate
    } else {
        return console.error("invalid parameters passed to calculateTax function")
    }
}

function calculateOrderTotal(subTotal, tax) {
    if ((Number(subTotal) && Number(tax))) {
        return subTotal + tax
    } else {
        return console.error("invalid parameters passed to calculateOrderTotal function")
    }
}
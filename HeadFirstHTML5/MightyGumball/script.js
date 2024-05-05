window.onload = () => init()

// Global variables
let lastReportTime = 0

const init = () => {
    setInterval(handleRefresh, 3000)
}

// const loadSalesData = () => {
//     // const url = "http://127.0.0.1:3000/MightyGumball/sales.json"
//     const url = "http://127.0.0.1:3002/sales.json"
//     // const url = "https://github.com/bethrobson/Head-First-HTML5/blob/master/chapter6/sales.json"
//     const type = 'GET'
//     loadData(url, type, updateSales)
// }

// const updateSales = (responseText) => {
const updateSales = (sales) => {
    const salesDiv = document.getElementById("sales")
    salesDiv.innerHTML = ''
    // const sales = JSON.parse(responseText)
    for (let i = 0; i < sales.length; i++) {
        const sale = sales[i]
        const div = document.createElement("div")
        div.setAttribute("class", "sale-item")
        const text = sale.name + " sold " + sale.sales + " gumballs"
        div.replaceChildren(document.createTextNode(text))
        salesDiv.appendChild(div)
    }
    if (sales.length > 0) {
        lastReportTime = sales[sales.length - 1].time
    }
}

const handleRefresh = () => {
    
    const baseUrl = "http://127.0.0.1:3002/data.jsonp"
    const callbackParam = "callback=updateSales"
    const timeParam = `lastReportTime=${lastReportTime}`
    const randomParam = `random=${new Date().getTime()}`
    const url = `${baseUrl}?${callbackParam}&${timeParam}&${randomParam}`
    const newScriptElement = document.createElement("script")
    newScriptElement.setAttribute("src", url)
    newScriptElement.setAttribute("id", "jsonp")
    
    const oldScriptElement = document.getElementById("jsonp")
    const headElement = document.getElementsByTagName("head")[0]
    if (oldScriptElement == null) {
        headElement.appendChild(newScriptElement)
    } else {
        headElement.replaceChild(newScriptElement, oldScriptElement)
    }
}

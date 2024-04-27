// window.onload = () => init()

// const init = () => {
//     loadSalesData()
// }

// const loadSalesData = () => {
//     // const url = "http://127.0.0.1:3000/MightyGumball/sales.json"
//     const url = "https://github.com/bethrobson/Head-First-HTML5/blob/master/chapter6/sales.json"
//     const type = 'GET'
//     loadData(url, type, updateSales)
// }

const updateSales = (responseText) => {
    const salesDiv = document.getElementById("sales")

    const sales = JSON.parse(responseText)
    for (let i = 0; i < sales.length; i++) {
        const sale = sales[i]
        const div = document.createElement("div")
        div.setAttribute("class", "sale-item")
        const text = sale.name + " sold " + sale.sales + " gumballs"
        div.replaceChildren(document.createTextNode(text))
        salesDiv.appendChild(div)
    }
}

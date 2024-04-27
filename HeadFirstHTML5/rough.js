const lib = require('./lib.js')

class Movie {
    constructor({title, genre, rating, showtimes}) {
        this.title = title
        this.genre = genre
        this.rating = rating
        this.showtimes = showtimes
    }

    printInfo() {
        console.log(`Title: ${this.title}, Genre: ${this.genre}, Rating: ${this.rating}`)
    }

    findNextShowing() {
        const currentTime = new Date().getTime()
        let lastShowtime = null

        for (let i = 0; i < this.showtimes.length; i++ ) {
            const showtime = lib.getTimeFromString(this.showtimes[i])
            if ((showtime - currentTime) > 0) {
                return showtime
            }
            lastShowtime = this.showtimes[i]
        }
        return lastShowtime

    }
    getNextShowing() {
        const nextShowing = this.findNextShowing()
        if (nextShowing) {
            return `Next showing of ${this.title} is ${nextShowing}`
        } else {
            return `There are no more showings. The last showing was at ${this.showtimes[this.showtimes.length - 1]}`
        }
    }    
}

const movie1 = new Movie(
    {
        title: "Plan 9 from Outer Space",
        genre: "cult classic",
        rating: 2,
        showtimes: ["03:00am", "03:00pm", "07:00pm", "11:00pm"]
    }
)

movie1.printInfo()
let resp = movie1.getNextShowing()
console.log(resp)
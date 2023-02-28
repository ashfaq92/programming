function Blog(date, body) {
    this.date = date
    this.body = body
}

const blogEntry = new Blog('12/12/23', 'lorem ipsum')

console.log(blogEntry);
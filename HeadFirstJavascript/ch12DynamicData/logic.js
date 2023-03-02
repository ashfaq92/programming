// Helper functions
function getById(id) {
    return document.getElementById(id)
}

function fetchData(method, url) {
    return new Promise(function(resolve, reject) {
        const xhr = new XMLHttpRequest()
        xhr.onreadystatechange = function() {
            if (this.readyState == XMLHttpRequest.DONE) {
                if (this.status == 200) {
                    const contentType = xhr.getResponseHeader("Content-Type")
                    if (contentType.indexOf("xml") != -1) {
                        resolve(xhr.responseXML)
                    } else {
                        reject(new Error('Invalid content type'))
                    }
                } else if (this.status == 404) {
                    reject(new Error('Not found'))
                }
            } else if (this.readyState == 1 || this.readyState == 2 || this.readyState == 3) {
                getById('blogsContainer').innerHTML = `<img class="loading-img" src="./images/loading_gif.gif" />`
            }
        }
        xhr.open(method, url, true)
        xhr.send() 
    })
}

// Classes
Date.prototype.shortFormat = function() {
    let year = this.getFullYear()
    let day = this.getDate()
    let month = this.getMonth() + 1
    // padding with zeros if needed
    if (day < 10) day = '0' + day
    if (month < 10) month = '0' + month
    return month + '/' + day + '/' + year
}

// class methods (cannot be accessed by objects)
Blog.blogSorter = function(blog1, blog2){
    return blog2.date - blog1.date
}

// class-owned methods
// Blog.prototype.signature = 'Puzzler Ruby'

Blog.prototype.showImage = function() {
    if (this.imageURL) {
        return `<img alt="blog-img" class="img-responsive" src="${this.imageURL}" />` 
    } else {
        return ''
    }
}

Blog.prototype.containsText = function(searchQuery) {
    return this.text.includes(searchQuery)
}

Blog.prototype.toString = function() {
    return '[' + this.date.shortFormat() + ']' + ' ' + this.text 
}

Blog.prototype.view = function() {
    alert(this.toString())
}

Blog.prototype.toHTML = function(highlight) {
    return `
    <div class="entry ${highlight ? 'normal-entry' : 'inverse-entry'}">
        <div class='left'>${this.showImage()}</div>
        <div class='right'>
            <strong>${this.date.shortFormat()}</strong>
            <p>${this.text}</p>
            <em>by ${this.signature}</em>
        </div>
    </div>`

}

function setBlogSignature(xmlDoc) {
    Blog.prototype.signature = xmlDoc.getElementsByTagName('author')[0].textContent
}

// Constructors
function Blog(text, date, imageURL) {
    this.text = text
    this.date = date ? new Date(date) : new Date()
    this.imageURL = imageURL // null is specified by default if imageURL isn't passed
}

// Blog utils

function showBlogs(num) {
    blogs.sort(Blog.blogSorter)
    num ? populateBlogs.slice(0, num) : populateBlogs(blogs)
}
function populateBlogs(blogs) {
    let target = getById('blogsContainer')

    if (blogs.length == 0) {
        target.innerHTML = "No blog to show"
    } else {
        let output = ''
        blogs.forEach( (blog, index) => {
            let highlight = index % 2 == 0
            output += blog.toHTML(highlight)
        })
        target.innerHTML = output
    }
}

function searchBlogs(searchQuery, blogs) {
    if (searchQuery == '') {
        populateBlogs(blogs.slice(0,3))
    } else {
        const filteredBlogs = blogs.filter(blog => blog.containsText(searchQuery))
        populateBlogs(filteredBlogs)
    }
   
}

function keepTrackOfSearch(blogs) {
    let searchField = getById('searchQuery')
    searchField.oninput = function() {
        searchBlogs(searchField.value, blogs)
    }
}

function viewRandBlog(blogs) {
    const randIndex = Math.floor(Math.random() * blogs.length)
    const randBlog = blogs[randIndex]
    randBlog.view()
}


function sortBlogs(blogs) {
    blogs.sort(Blog.blogSorter)
}

function populateInitialBlogs(blogs) {
    populateBlogs(blogs.slice(0,3))
}

function attachlisteners(blogs) {
    getById('showAllBlogsBtn').onclick = () => populateBlogs(blogs)
    getById('viewRandBlogBtn').onclick = () => viewRandBlog(blogs)
    getById('searchBlogForm').onsubmit = (event) => {
        event.preventDefault()
        let searchField = getById('searchQuery')
        searchBlogs(searchField.value)
    }
    
}

function makeBlogObjects(xmlDoc) {
    let blogs = []
    const entries = xmlDoc.getElementsByTagName('entry')
    const entriesArray = Array.from(entries).map(entry => {
        return {
            text: entry.getElementsByTagName("text")[0].textContent,
            date: entry.getElementsByTagName("date")[0].textContent,
            imageURL: entry.getElementsByTagName("imageURL")[0].textContent
        }
    })
    entriesArray.forEach(entry => {
        blogs.push(
            new Blog(entry.text, entry.date, entry.imageURL)
        )
    })
    return blogs
}

function enableElement(elemId) {
    getById(elemId).disabled = false
}

window.onload = function() {
    fetchData('GET', 'http://127.0.0.1:5500/HeadFirstJavascript/ch12DynamicData/blogsData.xml')
    .then(function(response) {
        const blogXmlDoc = response
        setBlogSignature(blogXmlDoc)
        const blogs = makeBlogObjects(blogXmlDoc)
        sortBlogs(blogs)
        populateInitialBlogs(blogs)
        attachlisteners(blogs)
        keepTrackOfSearch(blogs)
        enableElement('showAllBlogsBtn')
        enableElement('viewRandBlogBtn')
        enableElement('searchQuery')

    })
    .catch(function(error) {
        console.log(error)
    })
}
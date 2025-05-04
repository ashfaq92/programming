package bookstore.models;

import java.io.Serializable;

/**
 * Represents a book in the bookstore system
 */
public class Book implements Serializable {
    // Serial version UID for serialization
    private static final long serialVersionUID = 1L;

    // Book attributes
    private String title;
    private String author;
    private String isbn;
    private double price;
    private int quantity;

    /**
     * Constructor for Book
     * 
     * @param title    Book title
     * @param author   Book author
     * @param isbn     Book ISBN
     * @param price    Book price
     * @param quantity Available quantity
     */
    public Book(String title, String author, String isbn, double price, int quantity) {
        this.title = title;
        this.author = author;
        this.isbn = isbn;
        this.price = price;
        this.quantity = quantity;
    }

    // Getters and setters
    public String getTitle() {
        return title;
    }

    public void setTitle(String title) {
        this.title = title;
    }

    public String getAuthor() {
        return author;
    }

    public void setAuthor(String author) {
        this.author = author;
    }

    public String getIsbn() {
        return isbn;
    }

    public void setIsbn(String isbn) {
        this.isbn = isbn;
    }

    public double getPrice() {
        return price;
    }

    public void setPrice(double price) {
        this.price = price;
    }

    public int getQuantity() {
        return quantity;
    }

    public void setQuantity(int quantity) {
        this.quantity = quantity;
    }

    /**
     * Decreases the quantity by one when a book is sold
     * 
     * @return true if the book was available, false otherwise
     */
    public boolean sellOne() {
        if (quantity > 0) {
            quantity--;
            return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "Book [title=" + title + ", author=" + author + ", isbn=" + isbn +
                ", price=" + price + ", quantity=" + quantity + "]";
    }
}
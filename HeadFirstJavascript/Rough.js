function Person(name, age) {
    this.name = name;
    this.age = age;
    this.ciao = () => {
        console.log('ciao', this.name)
    }
}
  
Person.prototype.greet = function() {
    console.log(`Hello, my name is ${this.ciao} and I am ${this.age} years old.`);
  }
  
  const person1 = new Person('Alice', 25);
const person2 = new Person('Bob', 30);


person1.greet(); // Output: "Hello, my name is Alice and I am 25 years old."

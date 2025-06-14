in_range(T) 
    :- not is_colder_than(T) & not is_warmer_than(T).

is_colder_than(T) :- temperature(C) & threshold(DT) & (T - C) > DT. 

is_warmer_than(T) :- temperature(C) & threshold(DT) & (C - T) > DT. 


+!temperature(T): in_range(T)
	<- 	println("Temperature achieved: ",T).

+!temperature(T) : is_colder_than(T)
    <-  println("it is too cold -> heating...");
        startHeating; 
        !heat_until(T).

+!heat_until(T): in_range(T)
    <-  stopAirConditioner; 
        println("Temperature achieved ",T).  

+!heat_until(T): is_colder_than(T)
    <-  .wait(100);
        !heat_until(T).  

@heat_until_loop 
+!heat_until(T): is_warmer_than(T)
    <-  .wait(100);
        !temperature(T).

@start_cooling 
+!temperature(T) : is_warmer_than(T)
    <-  println("It is too hot -> cooling...");
        startCooling; 
        !cool_until(T).

@cool_until_stop 
+!cool_until(T): in_range(T)
    <-  stopAirConditioner; 
        println("Temperature achieved ",T).

@cool_until_cool 
+!cool_until(T): is_warmer_than(T)
    <-  .wait(100); 
        !cool_until(T).

+!cool_until(T): is_colder_than(T)
    <-  .wait(100);
        !temperature(T).

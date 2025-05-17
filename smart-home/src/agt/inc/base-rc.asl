// get the avg of all preferred temperatures send by users' PAs 
average_pt(T) :- .findall(UT, pref_temp(UT), LT) & LT \== [] & T = math.average(LT).

threshold(0.4).




// change the target temperature whenever a new preferred temperature is received
+pref_temp(UT)[source(Ag)]
    <-
        .println("New preference from ", Ag, " = ", UT);
        if (pref_temp(Y)[source(Ag)] & UT \== Y) {
            -pref_temp(Y)[source(Ag)];
        }
        ?average_pt(T);
        .drop_desire(temperature(_));
        println("Creating a new goal to set temperature to ", T);
        !temperature(T);
    .

// @p1[atomic]
// +temperature(C): preferred_temperature(T) & not in_range(T) & not .desire(temperature(_))
//     <-  
//         println("Reacting to temperature change");
//         !temperature(T).

// @p2[atomic]
// +preferred_temperature(T): temperature(C) & not in_range(T) & not .desire(temperature(_))
//     <-
//         println("Reacting to temperature preference change");
//         !temperature(T).

// @p3[atomic]
// +preferred_temperature(T): temperature(C) & not in_range(T) & .desire(temperature(T1)) & T1 \== T2
//     <-
//         println("Reacting to temperature preference change");
//         .drop_desire(temperature(T1));
//         stopAirConditioner;
//         !temperature(T);
//     .


{ include("temp_management.asl") }
{ include("$jacamo/templates/common-cartago.asl") }
{ include("$jacamo/templates/common-moise.asl") }
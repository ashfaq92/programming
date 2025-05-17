closest(X,[H|T],H) :- X > H.
closest(X,[H1,H2|T],H1)
   :- X < H1 & X > H2 & H1-X <= X-H2.
closest(X,[H1,H2|T],H2)
   :- X < H1 & X > H2 & H1-X > X-H2.
closest(X,[H],H).
closest(X,[H|T],V)
   :- closest(X,T,V).


// translation of the low, high, and medium temperature levels into internal agent scale
level_temp(low, 10).
level_temp(medium, 20).
level_temp(high, 30).

// inference of the preferred temperature level from the activity
pref_temp(T) :- activity(A) & preferred(A, L) & level_temp(L, T).
pref_temp(20).  // temp for none activity

!create_GUI.

+!create_GUI
    <-
        .my_name(Me);
        .concat(gui_, Me, ArtName);
        makeArtifact(ArtName, "gui.UserGUI", [], ArtId);
        focus(ArtId);
    .


+activity(A) : A \== "none"
    <-
        ?pref_temp(T);
        .print("New user activity ", A, " preferred temperature is ", T);
        .send(rc, tell, pref_temp(T));
    .


{ include("$jacamoJar/templates/common-cartago.asl") }
{ include("$jacamoJar/templates/common-moise.asl") }
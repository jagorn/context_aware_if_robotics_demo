#program base.

% Context Variables
time(morning).
time(noon).
time(afternoon).
time(night).

light(dark).
light(bright).

person(nardi).

% Context Rules

_light(dark) :- _time(night).

people_influence_radius(10) :- _light(dark).
people_influence_radius(5) :- not _light(dark).

people_influence_value(10).

region_influence_value(cafeteria, 3) :- _time(morning).
region_influence_value(corridor, 2) :- _time(morning).
region_influence_value(office, 7) :- _time(morning).

region_influence_value(cafeteria, 8) :- _time(noon).
region_influence_value(corridor, 5) :- _time(noon).
region_influence_value(office, 1) :- _time(noon).

region_influence_value(cafeteria, 1) :- _time(afternoon).
region_influence_value(corridor, 5) :- _time(afternoon).
region_influence_value(office, 4) :- _time(afternoon).

% context_area(shape, isScaled, CentreX, CentreY, Radius[only cicle], Side[rect only], Height[rect only], Weight)
context_area(ID, circle, 1, CX, CY, R, 0, 0, W) :- _person_at(ID, CX, CY), people_influence_radius(R), people_influence_value(W).
context_area(ID, rect, 0, CX, CY, 0, S, H, W) :- region(ID, Type, CX, CY, S, H), region_influence_value(Type, W).

#external _time(T): time(T).
#external _light(L): light(L).
#external _person_at(ID, 1..100, 1..100): person(ID).

#show context_area/9.
#show _time/1.
#show _light/1.
#show _person_at/3.
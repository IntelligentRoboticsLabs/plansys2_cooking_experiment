(define (problem cooking2)
(:domain cooking)
(:objects 
  eggs flour sugar - ingredient
  fridge_zone pantry_zone watertap_zone recharge_zone cooking_zone - zone
  r2d2 - robot
  cake - dish
)
(:init 
  (is_cooking_zone cooking_zone)
  (is_fridge_zone fridge_zone)
  (is_pantry_zone pantry_zone)
  (is_watertap_zone watertap_zone)
  (is_recharge_zone recharge_zone)

  (ingredient_at eggs fridge_zone)
  (ingredient_at flour pantry_zone)
  (ingredient_at sugar pantry_zone)

  (is_egg eggs)
  (is_flour flour)
  (is_sugar sugar)
  (is_cake cake)

  (robot_at r2d2 cooking_zone)
  (battery_full r2d2)
)

(:goal (and
  (dish_prepared cake)
  )
)

)

set instance eggs ingredient
set instance flour ingredient
set instance sugar ingredient
set instance fridge_zone zone
set instance pantry_zone zone
set instance watertap_zone zone
set instance recharge_zone zone
set instance cooking_zone zone

set instance r2d2 robot
set instance cake dish


set predicate  (is_cooking_zone cooking_zone)
set predicate  (is_fridge_zone fridge_zone)
set predicate  (is_pantry_zone pantry_zone)
set predicate  (is_watertap_zone watertap_zone)
set predicate  (is_recharge_zone recharge_zone)
set predicate  (ingredient_at eggs fridge_zone)
set predicate  (ingredient_at flour pantry_zone)
set predicate  (ingredient_at sugar pantry_zone)
set predicate  (is_egg eggs)
set predicate  (is_flour flour)
set predicate  (is_sugar sugar)
set predicate  (is_cake cake)
set predicate  (robot_at r2d2 cooking_zone)
set predicate  (battery_full r2d2)
set goal (and (dish_prepared cake))

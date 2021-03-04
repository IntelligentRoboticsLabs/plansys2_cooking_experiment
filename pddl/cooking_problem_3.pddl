(define (problem cooking4)
(:domain cooking)
(:objects 
  pasta water oil salt eggs flour sugar - ingredient
  fridge_zone pantry_zone watertap_zone recharge_zone cooking_zone - zone
  r2d2 - robot
  spaghetti cake - dish
)
(:init 
  (is_cooking_zone cooking_zone)
  (is_fridge_zone fridge_zone)
  (is_pantry_zone pantry_zone)
  (is_watertap_zone watertap_zone)
  (is_recharge_zone recharge_zone)

  (ingredient_at water watertap_zone)
  (ingredient_at pasta pantry_zone)
  (ingredient_at oil pantry_zone)
  (ingredient_at salt pantry_zone)
  (ingredient_at eggs fridge_zone)
  (ingredient_at flour pantry_zone)
  (ingredient_at sugar pantry_zone)

  (is_water water)
  (is_oil oil)
  (is_salt salt)
  (is_pasta pasta)
  (is_egg eggs)
  (is_flour flour)
  (is_sugar sugar)
  
  (is_cake cake)
  (is_spaghetti spaghetti)

  (robot_at r2d2 cooking_zone)
  (battery_full r2d2)
)

(:goal (and
  (dish_prepared spaghetti)
  (dish_prepared cake)
  )
)

)



;;set instance pasta1 pasta
;;set instance water1 water
;;set instance oil1 oil
;;set instance salt1 salt
;;set instance fridge_zone zone
;;set instance pantry_zone zone
;;set instance watertap_zone zone
;;set instance recharge_zone zone
;;set instance cooking_zone zone
;;
;;set instance r2d2 robot
;;set instance spaghetti1 spaghetti
;;
;;set predicate  (is_cooking_zone cooking_zone)
;;set predicate  (is_fridge_zone fridge_zone)
;;set predicate  (is_pantry_zone pantry_zone)
;;set predicate  (is_watertap_zone watertap_zone)
;;set predicate  (is_recharge_zone recharge_zone)
;;set predicate  (ingredient_at pasta1 fridge_zone)
;;set predicate  (ingredient_at water1 pantry_zone)
;;set predicate  (ingredient_at oil1 pantry_zone)
;;set predicate  (ingredient_at salt1 pantry_zone)
;;set predicate  (robot_at r2d2 cooking_zone)
;;set predicate  (battery_full r2d2)
;;
;;set goal (and (dish_prepared spaghetti1))
;;
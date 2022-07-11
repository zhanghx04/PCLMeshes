#!/bin/zsh

search_radius=100
mu=2.5
max_near_neighbor=100  
max_surface_angle=45    # degrees
min_angle=10            # degrees
max_angle=120           # degrees
normal_consistency=0    # 0-false 1-true

# ply_file_name=./courthouse-normals-10-thins.ply
# ply_file_name=./ABQ215full_normals.ply
ply_file_name=./ABQ215_courthouse_normals.ply

method=1

if [[ $method -eq 1 ]]
then
    for mu in {25..30..1}
    do
        for max_near_neighbor in {50..100..10}
        do
            echo $mu and $max_near_neighbor
            ./build/main ${ply_file_name} ${search_radius} ${mu} ${max_near_neighbor} ${max_surface_angle} ${min_angle} ${max_angle} ${normal_consistency}
        done
    done
else
    echo One Iteration
    ./build/main ${ply_file_name} ${search_radius} ${mu} ${max_near_neighbor} ${max_surface_angle} ${min_angle} ${max_angle} ${normal_consistency}
fi
echo All done

# ./build/main ${ply_file_name} ${search_radius} ${mu} ${max_near_neighbor} ${max_surface_angle} ${min_angle} ${max_angle} ${normal_consistency}
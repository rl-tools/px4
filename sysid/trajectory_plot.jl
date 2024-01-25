using DataFrames
using CSV
using Plots
using TimeSeries
using Interpolations
using DataFrames
using Dates
using ProgressBars


find_closest(arr, x) = findmin(abs.(arr .- x))[2]

data_path = "sysid/figure_data/log_105_2024-1-18-15-46-50.ulg_trajectory_tracking.csv"
data = CSV.read(data_path, DataFrame)

data_target = dropmissing(data[!, ["Column1", "rl_tools_command_target_position[0]", "rl_tools_command_target_position[1]"]])
target_times_raw = data_target[!, "Column1"]

x_target_raw = data_target[!, "rl_tools_command_target_position[0]"]
y_target_raw = data_target[!, "rl_tools_command_target_position[1]"]
x_offset = x_target_raw[1]
y_offset = y_target_raw[1]
z_offset = 0
x_target_hf = x_target_raw .- x_offset
y_target_hf = y_target_raw .- y_offset

data_real = dropmissing(data[!, ["Column1", ["vehicle_local_position_$i" for i in "xyz"]..., ["vehicle_local_position_v$i" for i in "xyz"]...]])
x_real_raw = data_real[!, "vehicle_local_position_x"]
y_real_raw = data_real[!, "vehicle_local_position_y"]
z_real_raw = data_real[!, "vehicle_local_position_z"]
x_real_hf = x_real_raw .- x_offset
y_real_hf = y_real_raw .- y_offset
z_real_hf = z_real_raw .- z_offset
real_times_raw = data_real[!, "Column1"]
speed_hf = sqrt.(sum.(eachrow((data_real[!, ["vehicle_local_position_v$i" for i in "xyz"]].^2))))
start_time = max(target_times_raw[1], real_times_raw[1]) + 0.1
target_times = target_times_raw .- start_time
real_times = real_times_raw .- start_time
end_time = min(target_times[end], real_times[end]) - 0.1

x_real_interp = linear_interpolation(real_times, x_real_hf, extrapolation_bc=Line())
y_real_interp = linear_interpolation(real_times, y_real_hf, extrapolation_bc=Line())
z_real_interp = linear_interpolation(real_times, z_real_hf, extrapolation_bc=Line())
x_target_interp = linear_interpolation(target_times, x_target_hf, extrapolation_bc=Line())
y_target_interp = linear_interpolation(target_times, y_target_hf, extrapolation_bc=Line())
speed_interp = linear_interpolation(real_times, speed_hf, extrapolation_bc=Line())

fps = 30
times = collect(0:1/fps:end_time)
x_real = x_real_interp.(times)
y_real = y_real_interp.(times)
z_real = z_real_interp.(times)
speed = speed_interp.(times)
x_target = x_target_interp.(times)
y_target = y_target_interp.(times)

x_max = max(maximum(x_real), maximum(x_target))
x_min = min(minimum(x_real), minimum(x_target))
y_max = max(maximum(y_real), maximum(y_target))
y_min = min(minimum(y_real), minimum(y_target))

margin = 0.05
y_lim_max, y_lim_min = (y_max - y_min)*margin + y_max, y_min - (y_max - y_min)*margin 
x_lim_max, x_lim_min = (x_max - x_min)*margin + x_max, x_min - (x_max - x_min)*margin


plt = plot(legend=:topright) #plot(size=(1000, 500))
plot!(plt, x_target.-x_offset, y_target.-y_offset, aspect_ratio=:equal, xlabel="x [m]", ylabel="y [m]", color=:black, linewidth=4, legend=false, colorbar=true, colorbar_title="speed [m/s]", xflip=true, label="target") # xflip to establish top down ENU view

plot!(plt, x_real.-x_offset, y_real.-y_offset, line_z=speed, linewidth=2, label="actual")

figure_name = splitpath(splitext(data_path)[begin])[end]
savefig(plt, "sysid/figures/trajectory_tracking $figure_name.pdf")

display(plt)


anim = @animate for i in ProgressBar(1:length(times))
    plot(ylims=(y_lim_min, y_lim_max), xlims=(x_lim_min, x_lim_max), legend=:topright, dpi=300)
    plot!(x_target, y_target, aspect_ratio=:equal, xlabel="x [m]", ylabel="y [m]", color=:black, linewidth=4, legend=false, colorbar=true, colorbar_title="speed [m/s]", xflip=true)
    plot!(x_real_interp.(0:1/fps:times[i]), y_real_interp.(0:1/fps:times[i]), line_z=speed[1:i], linewidth=4)
    scatter!([x_real_interp(times[i])], [y_real_interp(times[i])], color="#7DB9B6", markersize=10, markerstrokewidth=2)
end

mp4(anim, "trajectory_tracking_animation.mp4", fps=fps)



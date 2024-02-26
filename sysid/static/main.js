// import pyodide from 'https://cdn.jsdelivr.net/npm/pyodide@0.25.0/+esm'
const pyodide_url = './lib/pyodide/pyodide.mjs'

const debug = true

function createLabeledInput(container, labelText, inputType, inputId) {
    const label = document.createElement('label');
    label.textContent = labelText;
    label.htmlFor = inputId;
  
    const input = document.createElement('input');
    input.type = inputType;
    input.id = inputId;
    input.name = inputId;
  
    container.appendChild(label);
    container.appendChild(input);
  
    container.appendChild(document.createElement('br'));
    return input;
  }

async function main() {
    console.log("Downloading pyodide...")
    let pyodide_module = await import(pyodide_url);
    console.log("Loading pyodide...")
    let pyodide = await pyodide_module.loadPyodide();
    console.log("Loading micropip...")
    await pyodide.loadPackage('micropip')
    let micropip = pyodide.pyimport("micropip");
    console.log("Installing pyulog")
    await micropip.install([
        "pyulog",
        "scipy",
        "pandas",
        "numpy",
        "scikit-learn",
        "matplotlib",
        "tqdm"
    ])
    const rotor_x_displacement = 0.4179/2
    const rotor_y_displacement = 0.481332/2
    window.model = {
        gravity: 9.81,
        mass: 2.3 + 1.05,
        rotor_positions: [
            [ rotor_x_displacement, -rotor_y_displacement, 0],
            [-rotor_x_displacement,  rotor_y_displacement, 0],
            [ rotor_x_displacement,  rotor_y_displacement, 0],
            [-rotor_x_displacement, -rotor_y_displacement, 0]
        ],
        rotor_thrust_directions: [
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1]
        ],
        rotor_torque_directions: [
            [0, 0, -1],
            [0, 0, -1],
            [0, 0,  1],
            [0, 0,  1]
        ]
    }

    console.log("Loading process.py...")
    const process_file = await fetch('./process.py')
    pyodide.runPython(await process_file.text())
    console.log("Loading load_ulg.py...")
    const load_ulg_file = await fetch('./load_ulg.py')
    pyodide.runPython(await load_ulg_file.text())

    pyodide.runPython(`
    import js
    model = js.model
    model.rotor_positions = np.array(model.rotor_positions)
    model.rotor_thrust_directions = np.array(model.rotor_thrust_directions)
    model.rotor_torque_directions = np.array(model.rotor_torque_directions)
    `)

    console.log("Loading log.ulg...")
    const ulg_log_file = await (await fetch('./log.ulg')).arrayBuffer()
    const ulg_log_file_buffer = new Uint8Array(ulg_log_file)

    console.log("Writing log.ulg...")
    pyodide.FS.writeFile("/log.ulg", ulg_log_file_buffer);

    pyodide.runPython(`
        output_topic = "actuator_motors_mux"
        ulog_files = ["/log.ulg"]
        dfs_raw = [load_ulg(file) for file in ulog_files]
        grep = "actuator_motors_mux"
        print(str([col for col in dfs_raw[0].columns if grep in col]))
    `)
    pyodide.runPython(`
        import matplotlib
        import os
        # matplotlib.use("module://matplotlib.backends.html5_canvas_backend")
        # matplotlib.use("module://matplotlib_pyodide.wasm_backend")
        plot_activation = False
        for i_file, (file, df) in enumerate(zip(ulog_files, dfs_raw)):
            print(f"File {i_file}: {file}")
            plt.figure()
            # plt.title(f"file {i_file}: {file}")
            for i_motor in range(4):
                s = df[f"{output_topic}_control[{i_motor}]"].dropna()
                plt.plot(s.index, s, label=f"motor {i_motor}")
            plt.xlabel("Time [s]")
            plt.ylabel("Throttle [0-1]")
            plt.legend()
            if "rl_tools_multiplexer_status_active" in df.columns and plot_activation:
                s = df["rl_tools_multiplexer_status_active"].dropna()
                plt.plot(s.index, s, label="active")
            os.makedirs("/figures_raw", exist_ok=True)
            plt.savefig(f"/figures_raw/throttle_{i_file}.svg")
    `)
    const throttle_files = pyodide.FS.readdir("/figures_raw").filter((file) => file.startsWith("throttle_")).sort()
    const file_inputs = []
    for (const filename of throttle_files) {
        let file = pyodide.FS.readFile("/figures_raw/" + filename);
        const file_id = /throttle_(\d+).svg/.exec(filename)[1];
        let arrayBuffer = new Uint8Array(file).buffer;
        const throttle_figure_blob = new Blob([arrayBuffer], {type: 'image/svg+xml'});
        const url = URL.createObjectURL(throttle_figure_blob);
        const throttle_container = document.createElement('div');
        const img = document.createElement('img');
        img.src = url;
        throttle_container.appendChild(img);
        const time_range_container = document.createElement('div');
        const use_for_throttle_estimation = createLabeledInput(time_range_container, 'Use for throttle estimation', 'checkbox', 'use_for_throttle_estimation_' + file_id);
        const use_for_inertia_estimation = createLabeledInput(time_range_container, 'Use for inertia estimation', 'checkbox', 'use_for_inertia_estimation_' + file_id);
        file_inputs.push({
            "start": createLabeledInput(time_range_container, 'Start', 'number', 'start_time_input_' + file_id),
            "end": createLabeledInput(time_range_container, 'End', 'number', 'end_time_input_' + file_id),
            "use_for_throttle_estimation": use_for_throttle_estimation,
            "use_for_inertia_estimation": use_for_inertia_estimation
        })
        throttle_container.appendChild(time_range_container);
        document.body.appendChild(throttle_container);
    }
    const submit_button = document.createElement('button');
    submit_button.textContent = 'Crop';
    const data_time_ranges_pp = pyodide.runPython(`
        [[df.index.min(), df.index.max()] for df in dfs_raw]`)
    const data_time_ranges = data_time_ranges_pp.toJs();

    const input_tau_test_container = document.createElement('div');
    const input_tau_test = createLabeledInput(input_tau_test_container, 'Tau test', 'number', 'tau_test_input');
    input_tau_test.value = "0.03";
    document.body.appendChild(input_tau_test_container);

    let tau_test = null;
    let file_inputs_parsed = null
    let file_inputs_valid = false;
    if(debug){
        file_inputs_parsed = [{start: 10, end: 45, use_for_throttle_estimation: true, use_for_inertia_estimation: true}]
        file_inputs_valid = true;
        tau_test = 0.03;
    }
    while(!file_inputs_valid && !debug){
        await new Promise((resolve) => {
            submit_button.onclick = resolve;
            document.body.appendChild(submit_button);
        })
        file_inputs_parsed = []
        file_inputs_valid = true;
        for(let i=0; i < file_inputs.length; i++){
            const start = parseFloat(file_inputs[i].start.value);
            const end = parseFloat(file_inputs[i].end.value);
            if (isNaN(start) || isNaN(end)){
                alert("Invalid time range (file " + i + ")");
                file_inputs_valid = false;
                break;
            }
            if(data_time_ranges[i][0] > start || data_time_ranges[i][1] < end){
                alert("Time range out of bounds (file " + i + ")");
                file_inputs_valid = false;
                break;
            }
            file_inputs_parsed.push({start, end,
                use_for_throttle_estimation: file_inputs[i].use_for_throttle_estimation.checked
            });
        }
        if(file_inputs_parsed.filter((range) => range.use_for_throttle_estimation).length === 0){
            alert("At least one file must be used for throttle estimation");
            file_inputs_valid = false;
        }
        tau_test = parseFloat(input_tau_test.value);
        if (isNaN(tau_test)){
            alert("Invalid tau test");
            file_inputs_valid = false;
        }
    }
    console.log("File inputs: ", file_inputs_parsed);
    window.time_ranges = file_inputs_parsed.map((range) => [range.start, range.end]);
    window.tau_test = tau_test;
    pyodide.runPython(`
    import js
    dfs = [timeframe(df, time_start, time_end) for df, (time_start, time_end) in zip(dfs_raw, js.time_ranges)]
    `)

    pyodide.runPython(`
    import js
    tau_test = js.tau_test
    dfs_tt = [throttle_thrust(df, tau_test, model, output_topic) for df in dfs]
    for i, (file, df) in enumerate(zip(ulog_files, dfs_tt)):
        plt.figure()
        plt.title(f"File {i}: {file} tau: {tau_test}")
        plt.plot(df["throttle"].dropna(), label="throttle")
        plt.ylabel("Throttle [0-1]^2")
        plt.xlabel("Time [s]")
        plt.twinx()
        plt.plot(df["thrust"].dropna(), label="acceleration", color="orange")
        plt.ylabel("Thrust [N]")
        os.makedirs("/figures_time_range", exist_ok=True)
        plt.savefig(f"/figures_time_range/{i_file}.svg")
    `)

    const time_range_files = pyodide.FS.readdir("/figures_time_range").filter((file) => file.endsWith(".svg")).sort()
    for (const filename of time_range_files) {
        let file = pyodide.FS.readFile("/figures_time_range/" + filename);
        const file_id = /(\d+).svg/.exec(filename)[1];
        let arrayBuffer = new Uint8Array(file).buffer;
        const throttle_figure_blob = new Blob([arrayBuffer], {type: 'image/svg+xml'});
        const url = URL.createObjectURL(throttle_figure_blob);
        const time_range_container = document.createElement('div');
        const img = document.createElement('img');
        img.src = url;
        time_range_container.appendChild(img);
        document.body.appendChild(time_range_container);
    }

    const throttle_estimation_indices = file_inputs_parsed.map((range, i) => range.use_for_throttle_estimation ? i : null).filter((i) => i !== null);
    console.log("Throttle estimation indices: ", throttle_estimation_indices);
    window.throttle_estimation_files = throttle_estimation_indices;
    const inertia_estimation_indices = file_inputs_parsed.map((range, i) => range.use_for_inertia_estimation ? i : null).filter((i) => i !== null);
    console.log("Inertia estimation indices: ", inertia_estimation_indices);
    window.inertia_estimation_files = inertia_estimation_indices;


    pyodide.runPython(`
    tau_correlations = find_tau([dfs[i] for i in js.throttle_estimation_files], model, output_topic)
    tau_argmax = tau_correlations[:, 1].argmax()
    tau = tau_correlations[tau_argmax, 0]
    plt.figure()
    plt.plot(tau_correlations[:, 0], tau_correlations[:, 1])
    plt.vlines(tau, tau_correlations[:, 1].min(), tau_correlations[:, 1].max(), color="red", label=f"Max Correlation Tau = {tau:.3f}s")
    plt.xlabel("Tau [s]")
    plt.ylabel("Correlation (throttle, thrust)")
    plt.legend()
    os.makedirs("/figures_tau_correlation", exist_ok=True)
    plt.savefig(f"/figures_tau_correlation/tau_correlation.svg")
    `)

    const tau_correlation_file = pyodide.FS.readFile("/figures_tau_correlation/tau_correlation.svg");
    let tau_correlation_array_buffer = new Uint8Array(tau_correlation_file).buffer;
    const tau_correlation_figure_blob = new Blob([tau_correlation_array_buffer], {type: 'image/svg+xml'});
    const tau_correlation_figure_url = URL.createObjectURL(tau_correlation_figure_blob);
    const tau_correlation_container = document.createElement('div');
    const tau_correlation_figure_img = document.createElement('img');
    tau_correlation_figure_img.src = tau_correlation_figure_url;
    tau_correlation_container.appendChild(tau_correlation_figure_img);
    document.body.appendChild(tau_correlation_container);

    pyodide.runPython(`
    dfs_tt = [throttle_thrust(df, tau, model, output_topic) for df in [dfs[i] for i in js.throttle_estimation_files]]
    percentile = 0.05

    df_tt = pd.concat(dfs_tt)
    df_sysid = df_tt[["thrust", "throttle"]].dropna()
    thrust = df_sysid["thrust"]
    throttle = df_sysid["throttle"]

    acceleration = thrust / model.mass
    real_acceleration = (acceleration - model.gravity).abs()
    real_acceleration_percentile = real_acceleration.quantile(percentile)
    hovering_throttles = (throttle[real_acceleration < real_acceleration_percentile]/4) ** 0.5
    hovering_throttle = hovering_throttles.median()
    plt.figure()
    plt.title("Hovering Throttle Distribution")
    counts, bin_edges, patches = plt.hist(hovering_throttles, bins=100)
    plt.vlines(hovering_throttle, 0, max(counts), color="red", label=f"Median ({hovering_throttle:.3f})")
    plt.xlabel("Hovering Throttle [0-1]")
    plt.ylabel("Count")
    plt.legend()
    os.makedirs("/figures_hovering_throttle", exist_ok=True)
    plt.savefig(f"/figures_hovering_throttle/hovering_throttle_distribution.svg")
    print(f"Hovering throttle: {hovering_throttle} (per motor)")
    `)

    const hovering_throttle_file = pyodide.FS.readFile("/figures_hovering_throttle/hovering_throttle_distribution.svg");
    let hovering_throttle_array_buffer = new Uint8Array(hovering_throttle_file).buffer;
    const hovering_throttle_figure_blob = new Blob([hovering_throttle_array_buffer], {type: 'image/svg+xml'});
    const hovering_throttle_figure_url = URL.createObjectURL(hovering_throttle_figure_blob);
    const hovering_throttle_container = document.createElement('div');
    const hovering_throttle_figure_img = document.createElement('img');
    hovering_throttle_figure_img.src = hovering_throttle_figure_url;
    hovering_throttle_container.appendChild(hovering_throttle_figure_img);
    const hovering_throttle_message = document.createElement('div');
    hovering_throttle_message.textContent = "Hovering throttle (per motor): " + pyodide.globals.get("hovering_throttle");
    hovering_throttle_container.appendChild(hovering_throttle_message);
    document.body.appendChild(hovering_throttle_container);

    pyodide.runPython(`
    correlation, (slope, intercept) = fit_tau(dfs_tt, tau, model, output_topic)
    os.makedirs("/figures_thrust_curve", exist_ok=True)
    plot_thrust_curve(df_tt, model, output_topic, tau, slope, intercept, hovering_throttle, filename=f"/figures_thrust_curve/thrust_curve.svg", save_only=True)
    `)

    const thrust_curve_file = pyodide.FS.readFile("/figures_thrust_curve/thrust_curve.svg");
    let thrust_curve_array_buffer = new Uint8Array(thrust_curve_file).buffer;
    const thrust_curve_figure_blob = new Blob([thrust_curve_array_buffer], {type: 'image/svg+xml'});
    const thrust_curve_figure_url = URL.createObjectURL(thrust_curve_figure_blob);
    const thrust_curve_container = document.createElement('div');
    const thrust_curve_figure_img = document.createElement('img');
    thrust_curve_figure_img.src = thrust_curve_figure_url;
    thrust_curve_container.appendChild(thrust_curve_figure_img);
    document.body.appendChild(thrust_curve_container);


    pyodide.runPython(`
    I_x, I_y = fit_inertia([dfs[i] for i in js.inertia_estimation_files], model, output_topic, tau, slope, intercept)
    os.makedirs("/figures_angular_acceleration", exist_ok=True)
    plot_torque_angular_acceleration_curve([dfs[i] for i in js.inertia_estimation_files], model, output_topic, tau, slope, intercept, filename=f"/figures_angular_acceleration/angular_acceleration_curve", save_only=True, filetype="svg")
    `)

    const angular_acceleration_curve_x_file = pyodide.FS.readFile("/figures_angular_acceleration/angular_acceleration_curve_x.svg");
    let angular_acceleration_curve_x_array_buffer = new Uint8Array(angular_acceleration_curve_x_file).buffer;
    const angular_acceleration_curve_x_figure_blob = new Blob([angular_acceleration_curve_x_array_buffer], {type: 'image/svg+xml'});
    const angular_acceleration_curve_y_file = pyodide.FS.readFile("/figures_angular_acceleration/angular_acceleration_curve_y.svg");
    let angular_acceleration_curve_y_array_buffer = new Uint8Array(angular_acceleration_curve_y_file).buffer;
    const angular_acceleration_curve_y_figure_blob = new Blob([angular_acceleration_curve_y_array_buffer], {type: 'image/svg+xml'});
    const angular_acceleration_curve_x_figure_url = URL.createObjectURL(angular_acceleration_curve_x_figure_blob);
    const angular_acceleration_curve_container = document.createElement('div');
    const angular_acceleration_curve_x_figure_img = document.createElement('img');
    angular_acceleration_curve_x_figure_img.src = angular_acceleration_curve_x_figure_url;
    const angular_acceleration_curve_y_figure_url = URL.createObjectURL(angular_acceleration_curve_y_figure_blob);
    const angular_acceleration_curve_y_figure_img = document.createElement('img');
    angular_acceleration_curve_y_figure_img.src = angular_acceleration_curve_y_figure_url
    angular_acceleration_curve_container.appendChild(angular_acceleration_curve_x_figure_img);
    angular_acceleration_curve_container.appendChild(angular_acceleration_curve_y_figure_img);
    document.body.appendChild(angular_acceleration_curve_container);

  };
  main();
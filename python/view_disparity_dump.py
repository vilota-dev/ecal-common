import streamlit as st
import matplotlib.pyplot as plt
import json
import numpy as np

def is_outlier(d, m=3):
    return abs(d - np.mean(d)) > m * np.std(d)

def main():
    # ---------------------- Session State ---------------------- #
    if "data" not in st.session_state:
        st.session_state['data'] = None

    # ------------------ Sidebar Placeholders ------------------ #
    file_uploader_placeholder = st.sidebar.empty()  # Upload a JSON config
    st.sidebar.divider()
    st.sidebar.slider("Azimuth Range", -60, 60, (-2, 2), key='az-range')
    st.sidebar.slider("Elevation Range", -50, 50, (-2, 2), key='el-range')
    st.sidebar.slider("Elevation Range", 0.0, 20.0, (4.0, 6.0), step=0.1, key='dist-range')


    # ------------------ Main page Placeholders ------------------ #
    title_placeholder = st.empty()

    # ------------------ FILE UPLOADING SECTION ------------------ #
    # Upload distortion model (.json format)
    uploaded_file = file_uploader_placeholder.file_uploader('Upload a disparity dump', type=['json'])
    if uploaded_file is not None:
        data = json.load(uploaded_file)
        st.session_state['data'] = data

    # ------------------ MAIN SECTION ------------------ #
    title_placeholder.title("Disparity Dump Visualizer")

    if st.session_state['data'] is not None:
        session_data = st.session_state["data"]["dump"]
        value_dump = session_data["value_dump"]
        scale_factor = session_data["scale_factor"]

        azd = np.array(value_dump[0]["value"])        
        eld = np.array(value_dump[2]["value"])
        disd = np.array(value_dump[1]["value"])
        offd = np.array(value_dump[3]["value"])

        depths = scale_factor / disd
        xs = np.tan(azd * np.pi / 180) * depths
        ys = np.tan(eld * np.pi / 180) * depths
        dists = np.sqrt(np.square(xs) + 
                        np.square(ys) + 
                        np.square(depths))

        depthactual = scale_factor / (disd + offd)
        xsactual = np.tan(azd * np.pi / 180) * depthactual
        ysactual = np.tan(eld * np.pi / 180) * depthactual
        distsactual = np.sqrt(np.square(xsactual) + 
                            np.square(ysactual) + 
                            np.square(depthactual))

        # error
        errors = np.abs(dists - distsactual) / distsactual * 100

        azd_inrange = np.logical_and(azd < st.session_state["az-range"][1], azd > st.session_state["az-range"][0])
        eld_inrange = np.logical_and(eld < st.session_state["el-range"][1], eld > st.session_state["el-range"][0])
        dist_inrange = np.logical_and(dists < st.session_state["dist-range"][1], dists > st.session_state["dist-range"][0])
        inrange = np.logical_and(azd_inrange, eld_inrange)
        inrange = np.logical_and(inrange, dist_inrange)
        inrange = np.logical_and(inrange, ~is_outlier(errors))
        errorsinrange = errors[inrange]

        # plt errors against azimuth, and errors against elevation
        plt.subplot(3, 1, 1)
        plt.scatter(azd[inrange], errorsinrange, s=5)
        plt.xlabel("Azimuth")
        plt.ylabel("% Error")

        plt.subplot(3, 1, 2)
        plt.scatter(eld[inrange], errorsinrange, s=5)
        plt.xlabel("Elevation")
        plt.ylabel("% Error")

        plt.subplot(3, 1, 3)
        plt.scatter(dists[inrange], errorsinrange, s=5)
        plt.xlabel("Distance")
        plt.ylabel("% Error")

        plt.rcParams['figure.figsize'] = [10, 15]

        st.pyplot(plt)
        st.caption(f'mean error: {np.mean(errorsinrange)}')
        st.caption(f'std dev error: {np.std(errorsinrange)}')

# run with `streamlit run view_disparity_dump.py`
if __name__ == '__main__':
    main()

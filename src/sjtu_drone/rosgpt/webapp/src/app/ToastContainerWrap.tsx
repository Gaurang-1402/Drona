'use client'

import { ToastContainer } from "react-toastify"
import 'react-toastify/dist/ReactToastify.css';



export const ToastContainerWrap = () => {
    return (
        <ToastContainer className='z-[100]' autoClose={2000} />
    )
}

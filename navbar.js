import { useState } from 'react'
import Image from "next/image"
import cat1 from "../public/images/download.png"

function NavLink({to, children}) {
    return <a href={to} className={`mx-4`}>
        {children}
    </a>
}

function MobileNav({open, setOpen}) {
    return (
        <div className={`absolute top-0 right-0 h-screen w-screen bg-white transform ${open ? "-translate-x-0" : "-translate-x-full"} transition-transform duration-300 ease-in-out filter drop-shadow-md `}>
            <div className="flex flex-col ml-4">
                <a className="text-xl font-medium my-4" href="/home" onClick={() => setTimeout(() => {setOpen(!open)}, 100)}>
                    HOME
                </a>
                <a className="text-xl font-normal my-4" href="/competition" onClick={() => setTimeout(() => {setOpen(!open)}, 100)}>
                    ICRA COMPETITION
                </a>
                <a className="text-xl font-normal my-4" href="/about" onClick={() => setTimeout(() => {setOpen(!open)}, 100)}>
                    ABOUT US
                </a>
                <a className="text-xl font-normal my-4" href="/officers" onClick={() => setTimeout(() => {setOpen(!open)}, 100)}>
                    OFFICERS
                </a>
            </div>  
        </div>
    )
}

export default function Navbar() {
    
    const [open, setOpen] = useState(false)
    return (
        <nav className="flex filter drop-shadow-md bg-black px-12 py-12 h-20">
            <MobileNav open={open} setOpen={setOpen}/>
            <div className="navbar-toggler navbar-toggler-right w-full flex flex-row">
                <div className="text-white font-serif text-lg">
                    <NavLink to="/home">
                        HOME
                    </NavLink>
                    <NavLink to="/competition">
                        ICRA COMPETITION
                    </NavLink>
                    <NavLink to="/about">
                        ABOUT US
                    </NavLink>
                    <NavLink to="/officers">
                        OFFICERS
                    </NavLink>
                </div>
            </div>
            <div className="w-9/12 flex justify-end items-center">

                <div className="z-50 flex relative w-8 h-8 flex-col justify-between items-center md:hidden" onClick={() => {
                    setOpen(!open)
                }}>
                    {/* hamburger button */}
                    <span className={`h-1 w-full bg-white rounded-lg transform transition duration-300 ease-in-out ${open ? "rotate-45 translate-y-3.5" : ""}`} />
                    <span className={`h-1 w-full bg-white rounded-lg transition-all duration-300 ease-in-out ${open ? "w-0" : "w-full"}`} />
                    <span className={`h-1 w-full bg-white rounded-lg transform transition duration-300 ease-in-out ${open ? "-rotate-45 -translate-y-3.5" : ""}`} />
                </div>
            </div>
        </nav>
    )
}
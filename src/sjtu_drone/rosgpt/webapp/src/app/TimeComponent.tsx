'use client'
// Import necessary modules from the React library
import React, { useState, useEffect } from "react";

function TimeComponent() {
  // `useState` hook is used to initialize state variables for time, day name, month, and day
  const [time, setTime] = useState('');
  const [dayname, setDayName] = useState("");
  const [month, setMonth] = useState("");
  const [day, setDay] = useState("");

  // We use `useEffect` to update the time every second.
  useEffect(() => {  // This function returns the current time in the format "hh:mm:ss".
    function getFormattedTime() {
      const currentTime = new Date();
      return currentTime.toLocaleTimeString(window.navigator.language, {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      });
    }
    const intervalId = setInterval(() => {
      const formattedTime = getFormattedTime();
      setTime(formattedTime);
    }, 1000);

    // We use `clearInterval` to clear the interval when the component is unmounted.
    return () => clearInterval(intervalId);
  }, []);

  // We use `useEffect` to initialize the day name, month, and day when the component is mounted.
  useEffect(() => {
    const today = new Date();
    const dayIndex = today.getDay();
    const days = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
    const monthNames = [
      "Jan", "Feb", "Mar", "Apr", "May", "Jun",
      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    ];

    setDayName(days[dayIndex]);
    setMonth(monthNames[today.getMonth()]);
    setDay(String(today.getDate()).padStart(2, "0"));
  }, []);



  return (
    <div className="App">
      <h1>
        {time} • {dayname}, {month} {day}
      </h1>
    </div>
  );
}

// Export the `TimeComponent` component
export default TimeComponent;

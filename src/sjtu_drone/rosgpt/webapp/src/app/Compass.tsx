'use client'

import React, { useState, useEffect } from 'react';

const Compass = () => {
  const [heading, setHeading] = useState(38);

  const updateHeading = () => {
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition((position) => {
        const angle = position.coords.heading || 0;
        setHeading(angle);
      });
    }
  };

  useEffect(() => {
    const watchId = navigator.geolocation.watchPosition(updateHeading);

    return () => {
      navigator.geolocation.clearWatch(watchId);
    };
  }, []);

  const compassStyle = {
    transform: `rotate(${360 - heading}deg)`,
  };

  return (
    <div>
      <img src='/compass.png' alt='compass' className='w-full' style={compassStyle} />
    </div>
  );
};

export default Compass;

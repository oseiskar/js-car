function buildCarRenderer(car, trackCoords) {
  const carCoords = trackCoords.append('g');

  const carBody = carCoords
      .append('rect')
        .attr('x', -car.dim.width*0.5)
        .attr('y', -car.dim.length*0.5)
        .attr('width', car.dim.width)
        .attr('height', car.dim.length)
        .attr('fill', 'green');

  const wheels = [[true,-1], [true,1], [false,-1], [false,1]].map(([isFront, side]) => {
    const coords = carCoords.append('g');
    const wheelW = car.dim.width * 0.2;
    const wheelL = car.dim.width * 0.4;
    const rect = coords.append('rect')
      .attr('x', -wheelW*0.5)
      .attr('y', -wheelL*0.5)
      .attr('width', wheelW)
      .attr('height', wheelL);

    return {
      coords,
      rect,
      isFront,
      side,
      x: car.dim.width * 0.5 * side,
      y: car.dim.length * 0.5 * (isFront ? -1 : 1)
    };
  });

  return () => {
    const [rotLeft, rotRight] = car.frontWheelAngles;

    // car coordinate system
    carCoords.attr('transform', `
      translate(${car.pos[0]}, ${car.pos[1]})
      rotate(${car.rot/Math.PI*180.0 + 90})`);

    wheels.forEach(wheel => {
      const rot = wheel.isFront ? (wheel.side > 0 ? rotLeft : rotRight) : 0.0;
      wheel.coords.attr('transform', `
        translate(${wheel.x}, ${wheel.y})
        rotate(${rot/Math.PI*180.0})`);

      const slip = wheel.isFront ? car.slip.front : car.slip.back;
      const color = slip ? 'gray' : 'black';
      wheel.rect.attr('fill', color);
    });
  };
}

function buildTrackRenderer(track, svg, width, height) {
  const metricSize = track.size;
  const scale = Math.min(width / metricSize.width, height / metricSize.height);
  const [szX, szY] = [width, height];

  const trackCoords = svg.append('g').attr('transform', `translate(0, ${height})`)
    .append('g').attr('transform', 'scale(1,-1)')
    .append('g').attr('transform', `translate(${width/2}, ${height/2})`)
    .append('g').attr('transform', `scale(${scale})`);

  // boundary
  trackCoords.append('rect')
    .attr('x', -metricSize.width/2)
    .attr('y', -metricSize.height/2)
    .attr('width', metricSize.width)
    .attr('height', metricSize.height)
    .attr('stroke', 'black').attr('stroke-width', 1 / scale)
    //.attr('fill', '#f0f0f0');
    .attr('fill', 'none');

  // track
  const trackPoints = track.points; // close loop
  const trackLine = d3.line()
    .x(d => d[0]).y(d => d[1])
    .curve(d3.curveLinearClosed)(trackPoints);
  trackCoords.append('path')
    .attr('d', trackLine)
    .attr('stroke', '#dddddd')
    .attr('stroke-width', track.trackWidth) // meters
    .attr('fill', 'none');

  trackCoords.append('path')
    .attr('d', trackLine)
    .attr('stroke', '#ffff00')
    .attr('stroke-width', 2.0 / scale)
    .attr('stroke-dasharray', `0.2 0.3`)
    .attr('fill', 'none');

  const carRenderer = buildCarRenderer(track.car, trackCoords);
  return () => {
    carRenderer();
  };
}

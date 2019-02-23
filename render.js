function buildCarRenderer(car, trackCoords) {
  const carCoords = trackCoords.append('g');

  return () => {
    // car coordinate system
    carCoords.attr('transform', `
      translate(${car.pos[0]}, ${car.pos[1]})
      rotate(${car.rot/Math.PI*180.0 + 90})`);

    carCoords.selectAll('g').remove();
    const carParts = carCoords.append('g');

    // car body
    carParts
      .append('rect')
        .attr('x', -car.dim.width*0.5)
        .attr('y', -car.dim.length*0.5)
        .attr('width', car.dim.width)
        .attr('height', car.dim.length)
        .attr('fill', 'green');

    // wheels
    [true, false].forEach(isFront => {
      const endSign = isFront ? -1 : 1;
      const slip = isFront ? car.slip.front : car.slip.back;
      const color = slip ? 'gray' : 'black';
      const wheelW = car.dim.width * 0.2;
      const wheelL = car.dim.width * 0.4;
      const [rotLeft, rotRight] = car.frontWheelAngles;

      [-1,1].forEach(side => {
        const rot = isFront ? (side > 0 ? rotLeft : rotRight) : 0.0;
        carParts
          .append('g')
          .attr('transform', `
            translate(${car.dim.width*0.5*side}, ${car.dim.length*0.5*endSign})
            rotate(${rot/Math.PI*180.0})`)
          .append('rect')
            .attr('x', -wheelW*0.5)
            .attr('y', -wheelL*0.5)
            .attr('width', wheelW)
            .attr('height', wheelL)
            .attr('fill', color);
      });
    });
  };
}

function buildTrackRenderer(track, svg, width, height) {
  const scale = Math.min(width / track.size.width, height / track.size.height);
  const [szX, szY] = [width, height];
  const [origX, origY] = [szX/2, szY/2];

  const trackCoords = svg.append('g').attr('transform', `translate(0, ${height})`)
    .append('g').attr('transform', 'scale(1,-1)')
    .append('g').attr('transform', `translate(${width/2}, ${height/2})`)
    .append('g').attr('transform', `scale(${scale})`);

  const carRenderer = buildCarRenderer(track.car, trackCoords);
  return () => {
    carRenderer();
  };
}

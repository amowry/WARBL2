(function($process) {

    "use strict";

    var express      = require('express'),
        app          = express(),
        server       = require('http').createServer(app),
        cookieParser = require('cookie-parser'),
        csrf         = require('csurf');

    app.use(cookieParser());
    app.use(csrf({ cookie: true }));
    app.use(express.static(__dirname + '/..'));
    server.listen($process.env.PORT || 3507);

})(process);
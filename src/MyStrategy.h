#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

class MyStrategy : public Strategy
{
public:
    MyStrategy();

	virtual void act(model::Robot const& me, model::Rules const& rules, model::Game const& game, model::Action & action) override;
#ifdef PRINT
	virtual std::string custom_rendering() override;

	rapidjson::StringBuffer s;
	rapidjson::Writer<rapidjson::StringBuffer> writer;
#endif
};

#endif

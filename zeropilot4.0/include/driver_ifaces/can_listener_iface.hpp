#pragma once

struct CanardRxTransfer;

class ICanListener {
    protected:
        ICanListener() = default;

    public:
        virtual ~ICanListener() = default;

        // The transfer pointer is only valid during the call, don't keep it
        virtual void onTransfer(CanardRxTransfer *transfer) = 0;
};
